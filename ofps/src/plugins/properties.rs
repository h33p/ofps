//! Dynamic properties.

use std::ops::{Deref, DerefMut};

/// Object with custom properties.
pub trait Properties {
    /// Get available properties.
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![]
    }

    fn props(&mut self) -> Vec<(&str, Property)> {
        self.props_mut()
            .into_iter()
            .map(|(n, p)| (n, p.into()))
            .collect()
    }
}

/// Property with a lower and upper bound.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(::serde::Serialize, ::serde::Deserialize))]
pub struct BoundedProp<T> {
    pub val: T,
    pub min: T,
    pub max: T,
}

impl<T> Deref for BoundedProp<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.val
    }
}

impl<T> DerefMut for BoundedProp<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.val
    }
}

impl<T: Ord + Copy> BoundedProp<T> {
    /// Clamp the underlying value between the lower and upper bounds.
    pub fn clamp(&mut self) {
        self.val = std::cmp::min(std::cmp::max(self.val, self.min), self.max);
    }
}

impl<'a, T: Copy> From<BoundedPropMut<'a, T>> for BoundedProp<T> {
    fn from(BoundedPropMut { val, min, max }: BoundedPropMut<'a, T>) -> Self {
        Self {
            val: *val,
            min,
            max,
        }
    }
}

/// Describes the type of a property.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(::serde::Serialize, ::serde::Deserialize))]
pub enum Property {
    String(String),
    Bool(bool),
    Float(BoundedProp<f32>),
    Usize(BoundedProp<usize>),
}

impl<'a> From<PropertyMut<'a>> for Property {
    fn from(prop: PropertyMut<'a>) -> Self {
        match prop {
            PropertyMut::String(s) => Self::String(s.clone()),
            PropertyMut::Bool(b) => Self::Bool(*b),
            PropertyMut::Float(p) => Self::Float(p.into()),
            PropertyMut::Usize(p) => Self::Usize(p.into()),
        }
    }
}

/// Property with a lower and upper bound.
pub struct BoundedPropMut<'a, T> {
    pub val: &'a mut T,
    pub min: T,
    pub max: T,
}

impl<'a, T> Deref for BoundedPropMut<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.val
    }
}

impl<'a, T> DerefMut for BoundedPropMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.val
    }
}

impl<'a, T: Ord + Copy> BoundedPropMut<'a, T> {
    /// Clamp the underlying value between the lower and upper bounds.
    pub fn clamp(&mut self) {
        *self.val = std::cmp::min(std::cmp::max(*self.val, self.min), self.max);
    }
}

impl<'a, T: Copy> From<&'a mut BoundedProp<T>> for BoundedPropMut<'a, T> {
    fn from(prop: &'a mut BoundedProp<T>) -> Self {
        Self {
            val: &mut prop.val,
            min: prop.min,
            max: prop.max,
        }
    }
}

/// Describes the type of a property.
pub enum PropertyMut<'a> {
    String(&'a mut String),
    Bool(&'a mut bool),
    Float(BoundedPropMut<'a, f32>),
    Usize(BoundedPropMut<'a, usize>),
}

impl<'a> From<&'a mut Property> for PropertyMut<'a> {
    fn from(prop: &'a mut Property) -> Self {
        match prop {
            Property::String(s) => Self::String(s),
            Property::Bool(b) => Self::Bool(b),
            Property::Float(p) => Self::Float(p.into()),
            Property::Usize(p) => Self::Usize(p.into()),
        }
    }
}

impl<'a> PropertyMut<'a> {
    /// Create a string property.
    ///
    /// # Arguments
    ///
    /// * `s` - reference to the underlying string to be mutated.
    pub fn string(s: &'a mut String) -> Self {
        Self::String(s)
    }

    /// Create a boolean property.
    ///
    /// # Arguments
    ///
    /// * `b` - reference to the underlying boolean to be mutated.
    pub fn bool(b: &'a mut bool) -> Self {
        Self::Bool(b)
    }

    /// Create a floating point property.
    ///
    /// # Arguments
    ///
    /// * `val` - reference to the underlying float to be mutated.
    /// * `min` - lowest value for the property.
    /// * `max` - highest value for the property.
    pub fn float(val: &'a mut f32, min: f32, max: f32) -> Self {
        Self::Float(BoundedPropMut { val, min, max })
    }

    /// Create an integer point property.
    ///
    /// # Arguments
    ///
    /// * `val` - reference to the underlying usize to be mutated.
    /// * `min` - lowest value for the property.
    /// * `max` - highest value for the property.
    pub fn usize(val: &'a mut usize, min: usize, max: usize) -> Self {
        Self::Usize(BoundedPropMut { val, min, max })
    }

    pub fn set(&mut self, other: &Property) {
        match (self, other) {
            (Self::String(s), Property::String(os)) => **s = os.clone(),
            (Self::Bool(b), Property::Bool(ob)) => **b = *ob,
            (Self::Float(val), Property::Float(oval)) => *val.val = oval.val,
            (Self::Usize(val), Property::Usize(oval)) => *val.val = oval.val,
            _ => {}
        }
    }
}
