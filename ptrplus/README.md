[![Build status](https://github.com/tmccombs/ptrplus/workflows/Build/badge.svg)](https://github.com/tmccombs/ptrplus/actions?query=workflow%3ABuild)
[![Crates.io](https://img.shields.io/crates/v/ptrplus.svg)](https://crates.io/crates/ptrplus)
[![Crates.io](https://img.shields.io/crates/d/ptrplus.svg)](https://crates.io/crates/ptrplus)

# ptrplus

Ptrplus is a library that adds additional functionality around raw pointers.

## Conversions

Ptrplus provides traits to convert between raw pointers and safer Rust pointer types. `AsPtr`, `IntoRaw`,
and `FromRaw` provide common traits for types that implement `as_ptr`, `into_raw`, and `from_raw` respectively.
Of note, these traits also have implementations for `Option` to handle nullable raw pointers.

## Examples

```rust
use ptrplus::AsPtr;

let x: &u32 = &5;
let y: *const u32 = x.as_ptr();
unsafe {
    assert_eq!(*y, 5);
}
```

```rust
use ptrplus::AsPtr;

let x = 5;
let o1: Option<&u32> = None;
let o2: Option<&u32> = Some(&x);

assert!(o1.as_ptr().is_null());
assert!(!o2.as_ptr().is_null());
unsafe {
    assert_eq!(*o2.as_ptr(), 5);
}
```

```rust
use ptrplus::IntoRaw;

let x: Box<u32> = Box::new(5);
let y: *mut u32 = IntoRaw::into_raw(x);
unsafe {
  assert_eq!(*y, 5);
  *y = 6;
  Box::from_raw(y);
}

```

```rust
use ptrplus::{FromRaw, IntoRaw};

let o1: Option<Box<u32>> = None;
let o2: Option<Box<u32>> = Some(Box::new(5));

let p1: *mut u32 = o1.into_raw();
let p2: *mut u32 = o2.into_raw();

assert!(p1.is_null());
assert!(!p2.is_null());
unsafe {
    assert_eq!(*p2, 5);
    let o1: Option<Box<u32>> = Option::from_raw(p1);
    let o2: Option<Box<u32>> = Option::from_raw(p2);
    assert!(o1.is_none());
    assert!(!o2.is_none());
}
```

