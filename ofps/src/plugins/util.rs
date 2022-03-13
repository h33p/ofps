use anyhow::{anyhow, Result};

use std::mem::MaybeUninit;
use std::path::Path;

#[cfg(not(any(target_os = "windows", target_os = "macos")))]
pub fn find_export_by_prefix(path: impl AsRef<Path>, prefix: &str) -> Result<Vec<String>> {
    use goblin::elf::Elf;

    let buffer = std::fs::read(path.as_ref())?;
    let elf = Elf::parse(buffer.as_slice())?;

    Ok(elf
        .syms
        .iter()
        .filter_map(|s| {
            if let Some(name) = elf.strtab.get_at(s.st_name) {
                match name.starts_with(prefix) {
                    true => Some(name.to_owned()),
                    false => None,
                }
            } else {
                None
            }
        })
        .collect::<Vec<_>>())
}

#[cfg(target_os = "windows")]
pub fn find_export_by_prefix(
    path: impl AsRef<Path>,
    prefix: &str,
) -> crate::error::Result<Vec<String>> {
    use goblin::pe::PE;

    let buffer = std::fs::read(path.as_ref())?;
    let pe = PE::parse(buffer.as_slice())?;

    Ok(pe
        .exports
        .iter()
        .filter_map(|s| s.name)
        .filter_map(|name| {
            if name.starts_with(prefix) {
                Some(name.to_owned())
            } else {
                None
            }
        })
        .collect::<Vec<_>>())
}

#[cfg(target_os = "macos")]
pub fn find_export_by_prefix(
    path: impl AsRef<Path>,
    prefix: &str,
) -> crate::error::Result<Vec<String>> {
    use goblin::mach::Mach;

    let buffer = std::fs::read(path.as_ref())?;
    let mach = Mach::parse(buffer.as_slice())?;

    let macho = match mach {
        Mach::Binary(mach) => mach,
        Mach::Fat(mach) => (0..mach.narches)
            .filter_map(|i| mach.get(i).ok())
            .next()
            .ok_or_else(|| anyhow!("failed to find valid MachO header!"))?,
    };

    // macho symbols are prefixed with `_` in the object file.
    let macho_prefix = "_".to_owned() + prefix;
    Ok(macho
        .symbols
        .ok_or_else(|| anyhow!("failed to parse MachO symbols!"))?
        .iter()
        .filter_map(|s| s.ok())
        .filter_map(|(name, _)| {
            // symbols should only contain ascii characters
            if name.starts_with(&macho_prefix) {
                Some(name[1..].to_owned())
            } else {
                None
            }
        })
        .collect::<Vec<_>>())
}
