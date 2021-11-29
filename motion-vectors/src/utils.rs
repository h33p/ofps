pub trait AsMutPtr {
    type Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut;
}

impl<T: AsMutPtr> AsMutPtr for Option<T> {
    type Mut = T::Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}

impl<T> AsMutPtr for &mut T {
    type Mut = T;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}

impl<T: AsMutPtr> AsMutPtr for *mut T {
    type Mut = T::Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}
