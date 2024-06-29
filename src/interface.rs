use nalgebra::{ArrayStorage, Matrix, Vector, U2, U9};

pub trait Vectorizable<T, D, S> {
    fn to_vector(&self) -> Vector<T, D, S>;
}

pub type Matrix2x9<T> = Matrix<T, U2, U9, ArrayStorage<T, 2, 9>>;

pub trait WithRestriction<T> {
    fn generate_restriction(&self) -> Matrix2x9<T>;
}
