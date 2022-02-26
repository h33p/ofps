pub mod multiview;

#[allow(warnings)]
mod sys {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
