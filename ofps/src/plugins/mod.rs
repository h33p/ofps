//! # OFPS plugin system
//!
//! The plugin system allows to dynamically load implementations of various interfaces.
//!
//! Adapted from our previous work on [`memflow`](https://github.com/memflow/memflow/blob/main/memflow/src/plugins/mod.rs).
//!
//! The key difference here is that we are not performing runtime ABI stability checks. Instead, we
//! rely on compiler versions perfectly matching. A plugin stores the version in the `Descriptor`
//! structure. We do this, because we are relying too much on rust-native libraries, such as
//! `nalgebra` and `anyhow`. In addition, loaded plugin libraries can not be unloaded. This is
//! specifically because of `anyhow` - error propagation could yield data in unloaded libraries.
//! Due to us not using CGlue's integer errors, it is simply unsafe to unload the shared libraries.

use anyhow::{anyhow, Result};
use cglue::slice::CSliceRef;
use libloading::Library;
use log::*;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::{decoder::Decoder, detection::Detector, estimator::Estimator};

pub mod properties;
mod util;

use properties::Properties;

mod version {
    // Include a version source file generated by build.rs
    include!(concat!(env!("OUT_DIR"), "/version.rs"));
}
pub use version::RUSTC_VERSION;

/// OFPS API version used to ensure compatibility.
pub const API_VERSION: i32 = 1;

/// Plugin descriptor structure.
///
/// This structure is the base description of the plugin. It contains all the information necessary
/// to check for its compatibility and instantiation.
#[repr(C)]
pub struct Descriptor<T: Loadable> {
    pub api_version: i32,
    pub compiler: CSliceRef<'static, u8>,
    pub name: CSliceRef<'static, u8>,
    pub create: CreateFn<T>,
}

/// Define a plugin.
///
/// This macro creates a `$trait` type plugin under the name `$name`. `$create_fn` is a closure
/// that must return `Box<dyn $trait>`, and accepts the arguments type of the `$trait` plugin.
#[macro_export]
macro_rules! define_descriptor {
    ($name:ident, $trait:ident, $create_fn:expr) => {
        mod __ofps_descriptor {
            use super::*;
            use $crate::plugins as __plugins;

            $crate::paste::paste!{
                use __plugins::[<$trait Plugin>] as OfpsPluginType;
                use __plugins::[<$trait PluginTrait>] as OfpsPluginTrait;
            }

            #[allow(unused_braces)]
            fn __create_fn(args: <OfpsPluginType as __plugins::Loadable>::ArgsType) -> Result<OfpsPluginType> {
                fn __call_closure<T>(
                    __args: T,
                    __closure: impl Fn(T) -> Result<OfpsPluginType>
                ) -> Result<OfpsPluginType> {
                    __closure(__args)
                }

                __call_closure(args, $create_fn)
            }

            $crate::paste::paste!{
                #[no_mangle]
                pub static [<OFPS_ $trait _ $name>]: __plugins::Descriptor<Box<dyn OfpsPluginTrait + Send + 'static>> = __plugins::Descriptor {
                    api_version: __plugins::API_VERSION,
                    compiler: $crate::cglue::slice::CSliceRef::from_str(__plugins::RUSTC_VERSION),
                    name: $crate::cglue::slice::CSliceRef::from_str(stringify!($name)),
                    create: __create_fn,
                };
            }
        }
    };
}

impl<T: Loadable> Descriptor<T> {
    /// Get the name of the plugin.
    fn name(&self) -> &str {
        unsafe { self.name.into_str() }
    }

    /// Creates a new loadable instance from this library.
    ///
    /// The decoder is initialized with the arguments provided to this function.
    fn instantiate(&self, args: T::ArgsType) -> Result<T> {
        (self.create)(args)
    }

    /// Checks if plugin with the same `ident` already exists in input list
    fn exists(&self, instances: &[LibInstance<T>]) -> bool {
        instances.iter().any(|i| i.loader.name() == self.name())
    }
}

pub type CreateFn<T> = fn(<T as Loadable>::ArgsType) -> Result<T>;

/// Reference counted library instance.
///
/// This stores the necessary reference counted library instance, in order to prevent the library
/// from unloading unexpectedly. This is the required safety guarantee.
#[repr(C)]
pub struct LibInstance<T: Loadable> {
    path: PathBuf,
    library: Arc<Library>,
    loader: Descriptor<T>,
}

/// Base trait for loadable plugins.
pub trait Loadable: Sized {
    type ArgsType;

    /// Type of the plugin. Typically identifier of the trait.
    fn plugin_type() -> &'static str;

    /// Constant prefix for the plugin type
    fn export_prefix() -> &'static str;

    /// Typecheck and load up the plugin.
    fn load(
        path: impl AsRef<Path>,
        library: &Arc<Library>,
        export: &str,
    ) -> Result<LibInstance<Self>> {
        // find os descriptor
        let descriptor = unsafe {
            library
                .get::<*mut Descriptor<Self>>(format!("{}\0", export).as_bytes())?
                .read()
        };

        // check version
        if descriptor.api_version != API_VERSION {
            Err(anyhow!(
                "{} has a different version. version {} required, found {}.",
                export,
                API_VERSION,
                descriptor.api_version
            ))
        } else if &*descriptor.compiler != RUSTC_VERSION.as_bytes() {
            Err(anyhow!(
                "{} was built on different compiler version. {} expected, found {}.",
                export,
                RUSTC_VERSION,
                unsafe { descriptor.compiler.into_str() }
            ))
        } else {
            Ok(LibInstance {
                path: path.as_ref().to_path_buf(),
                library: library.clone(),
                loader: descriptor,
            })
        }
    }

    /// Try to load a plugin library
    ///
    /// This function will access `library` and try to find corresponding entry for the plugin. If
    /// a valid plugins are found, `Ok(LibInstance<Self>)` is returned. Otherwise, `Err(Error)` is
    /// returned, with appropriate error.
    ///
    /// # Safety
    ///
    /// Loading third party libraries is inherently unsafe and the compiler
    /// cannot guarantee that the implementation of the library
    /// matches the one specified here. This is especially true if
    /// the loaded library implements the necessary interface manually.
    ///
    /// It is adviced to use a provided proc macro to define a valid library.
    fn load_all(path: impl AsRef<Path>) -> Result<Vec<LibInstance<Self>>> {
        let exports = util::find_export_by_prefix(path.as_ref(), Self::export_prefix())?;
        if exports.is_empty() {
            return Err(anyhow!("No exports found"));
        }

        // load library
        let library = unsafe { Library::new(path.as_ref()) }
            .map_err(|err| {
                anyhow!(
                    "found {:?} in library '{:?}' but could not load it: {}",
                    exports,
                    path.as_ref(),
                    err
                )
            })
            .map(Arc::from)?;

        exports
            .into_iter()
            .map(|e| Self::load(path.as_ref(), &library, &e))
            .collect()
    }

    /// Helper function to load a plugin into a list of library instances
    ///
    /// This function will try finding appropriate plugin entry, and add it into the list if there
    /// isn't a duplicate entry.
    ///
    /// # Safety
    ///
    /// Loading third party libraries is inherently unsafe and the compiler
    /// cannot guarantee that the implementation of the library matches the one
    /// specified here.
    fn load_append(path: impl AsRef<Path>, out: &mut Vec<LibInstance<Self>>) -> Result<()> {
        let libs = Self::load_all(path.as_ref())?;
        for lib in libs.into_iter() {
            if !lib.loader.exists(out) {
                info!(
                    "adding plugin '{}/{}': {:?}",
                    Self::plugin_type(),
                    lib.loader.name(),
                    path.as_ref()
                );
                out.push(lib);
            } else {
                return Err(anyhow!(
                    "skipping library '{}' because it was added already: {:?}",
                    lib.loader.name(),
                    path.as_ref()
                ));
            }
        }

        Ok(())
    }
}

pub trait DecoderPluginTrait: Decoder + Properties {}
impl<T: Decoder + Properties> DecoderPluginTrait for T {}

pub type DecoderPlugin = Box<dyn DecoderPluginTrait + Send + 'static>;

impl Loadable for DecoderPlugin {
    type ArgsType = String;

    fn export_prefix() -> &'static str {
        "OFPS_Decoder_"
    }

    fn plugin_type() -> &'static str {
        "Decoder"
    }
}

pub trait EstimatorPluginTrait: Estimator + Properties {}
impl<T: Estimator + Properties> EstimatorPluginTrait for T {}

pub type EstimatorPlugin = Box<dyn EstimatorPluginTrait + Send + 'static>;

impl Loadable for EstimatorPlugin {
    type ArgsType = String;

    fn export_prefix() -> &'static str {
        "OFPS_Estimator_"
    }

    fn plugin_type() -> &'static str {
        "Estimator"
    }
}

pub trait DetectorPluginTrait: Detector + Properties {}
impl<T: Detector + Properties> DetectorPluginTrait for T {}

pub type DetectorPlugin = Box<dyn DetectorPluginTrait + Send + 'static>;

impl Loadable for DetectorPlugin {
    type ArgsType = String;

    fn export_prefix() -> &'static str {
        "OFPS_Detector_"
    }

    fn plugin_type() -> &'static str {
        "Detector"
    }
}

/// Dynamic plugin store.
///
/// This structure scans various directories upon its construction and contains a list of various
/// plugins that could be loaded.
pub struct PluginStore {
    decoders: Vec<LibInstance<DecoderPlugin>>,
    estimators: Vec<LibInstance<EstimatorPlugin>>,
    detectors: Vec<LibInstance<DetectorPlugin>>,
}

impl Default for PluginStore {
    fn default() -> Self {
        Self::new()
    }
}

impl PluginStore {
    /// Create a new `PluginStore`.
    pub fn new() -> Self {
        #[cfg(unix)]
        let extra_paths: Vec<&str> = vec!["/opt", "/lib"];
        #[cfg(not(unix))]
        let extra_paths: Vec<&str> = vec![];

        let path_iter = extra_paths.into_iter().map(PathBuf::from);

        // add user directory
        #[cfg(unix)]
        let path_iter = path_iter.chain(
            dirs::home_dir()
                .map(|dir| dir.join(".local").join("lib"))
                .into_iter(),
        );

        #[cfg(not(unix))]
        let path_iter = path_iter.chain(dirs::document_dir().into_iter());

        let mut ret = Self {
            decoders: vec![],
            estimators: vec![],
            detectors: vec![],
        };

        for mut path in path_iter {
            path.push("ofps");
            ret.scan_dir(path).ok();
        }

        // add current working directory
        if let Ok(pwd) = std::env::current_dir() {
            ret.scan_dir(pwd).ok();
        }

        // add directory of current executable
        if let Ok(mut exe) = std::env::current_exe() {
            exe.pop();
            ret.scan_dir(exe).ok();
        }

        ret
    }

    /// Scan a directory and collect all plugins inside of it.
    pub fn scan_dir(&mut self, dir: PathBuf) -> Result<&mut Self> {
        if !dir.is_dir() {
            return Err(anyhow!("Invalid path"));
        }

        info!("scanning {:?} for plugins", dir);

        for entry in std::fs::read_dir(dir)? {
            self.load(entry?.path());
        }

        Ok(self)
    }

    /// Adds a single library to the store
    ///
    /// # Safety
    ///
    /// Same as previous functions - compiler can not guarantee the safety of
    /// third party library implementations.
    pub fn load(&mut self, path: PathBuf) -> &mut Self {
        Loadable::load_append(&path, &mut self.decoders).ok();
        Loadable::load_append(&path, &mut self.estimators).ok();
        Loadable::load_append(&path, &mut self.detectors).ok();
        self
    }

    /// Returns the names of all currently available decoders that can be used.
    pub fn available_decoders(&self) -> Vec<String> {
        self.decoders
            .iter()
            .map(|c| c.loader.name().to_string())
            .collect::<Vec<_>>()
    }

    /// Returns the names of all currently available estimators that can be used.
    pub fn available_estimators(&self) -> Vec<String> {
        self.estimators
            .iter()
            .map(|c| c.loader.name().to_string())
            .collect::<Vec<_>>()
    }

    /// Returns the names of all currently available detectors that can be used.
    pub fn available_detectors(&self) -> Vec<String> {
        self.detectors
            .iter()
            .map(|c| c.loader.name().to_string())
            .collect::<Vec<_>>()
    }

    /// Create a new instance of a decoder.
    pub fn create_decoder(&self, name: &str, args: String) -> Result<DecoderPlugin> {
        Self::create_internal(&self.decoders, name, args)
    }

    /// Create a new instance of motion estimator.
    pub fn create_estimator(&self, name: &str, args: String) -> Result<EstimatorPlugin> {
        Self::create_internal(&self.estimators, name, args)
    }

    /// Create a new instance of motion detector.
    pub fn create_detector(&self, name: &str, args: String) -> Result<DetectorPlugin> {
        Self::create_internal(&self.detectors, name, args)
    }

    fn create_internal<T: Loadable>(
        libs: &[LibInstance<T>],
        name: &str,
        args: T::ArgsType,
    ) -> Result<T> {
        let lib = libs
            .iter()
            .find(|c| c.loader.name() == name)
            .ok_or_else(|| {
                anyhow!(
                    "unable to find plugin with name '{}'. available `{}` plugins are: {}",
                    name,
                    T::plugin_type(),
                    libs.iter()
                        .map(|c| c.loader.name().to_string())
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            })?;

        info!(
            "attempting to load `{}` type plugin `{}` from `{}`",
            T::plugin_type(),
            lib.loader.name(),
            lib.path.to_string_lossy(),
        );

        // Leak off the library so it never gets unloaded
        let _ = Arc::into_raw(lib.library.clone());

        lib.loader.instantiate(args)
    }
}
