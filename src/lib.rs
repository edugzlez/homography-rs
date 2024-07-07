// Module: homography_rs
// File: src/lib.rs

//! A library for computing homography using point and line correspondences.
//!
//! # Example
//!
//! ```
//! use homography::{HomographyComputation};
//! use homography::geo::{Point, Line};
//!
//! // Create a new instance of HomographyComputation
//!
//! let mut hc = HomographyComputation::new();
//!
//! // Define points
//!
//! let p1 = Point::new(148., 337.);
//! let p2 = Point::new(131., 516.);
//! let p3 = Point::new(321., 486.);
//! let p4 = Point::new(332., 370.);
//!
//! let p1p = Point::new(0., 0.);
//! let p2p = Point::new(0., 60.);
//! let p3p = Point::new(80., 60.);
//! let p4p = Point::new(80., 0.);
//!
//! // Define lines
//!
//! let line1 = Line::from_points(&p1, &p2);
//! let line1p = Line::from_points(&p1p, &p2p);
//!
//! // Add point and line correspondences
//!
//! hc.add_point_correspondence(p1, p1p);
//! hc.add_point_correspondence(p2, p2p);
//! hc.add_point_correspondence(p3, p3p);
//! hc.add_point_correspondence(p4, p4p);
//!
//! hc.add_line_correspondence(line1, line1p);
//!
//! // Get restrictions and compute solution
//!
//! let restrictions = hc.get_restrictions();
//!
//! let solution = restrictions.compute();
//!
//! // Print the solution
//!
//! println!("Matrix: {}", solution.matrix);
//! println!("Value: {}", solution.value);

pub mod functions;
pub mod geo;
pub mod interface;
pub mod pairs;

use crate::functions::{solve, HomographySolution};
use crate::geo::{Line, Point};
use crate::interface::{Matrix2x9, WithRestriction};
use crate::pairs::LinePair;
use nalgebra::{DMatrix, RealField, Scalar};

use crate::pairs::PointPair;

/// Represents restrictions for a homography computation.
/// The restrictions are represented as a vector of 2x9 matrices.
pub struct HomographyRestrictions<T: Scalar> {
    restrictions: Vec<Matrix2x9<T>>,
}

/// Represents a homography computation, which involves finding a transformation matrix
/// that maps points and lines from one coordinate system to another.
pub struct HomographyComputation {
    point_correspondences: Vec<PointPair>,
    line_correspondences: Vec<LinePair>,
}

/// Implementation of HomographyComputation, which represents a computation of homography.
impl HomographyComputation {
    /// Creates a new instance of HomographyComputation.
    ///
    /// # Returns
    ///
    /// A new instance of HomographyComputation.
    pub fn new() -> Self {
        HomographyComputation {
            point_correspondences: Vec::new(),
            line_correspondences: Vec::new(),
        }
    }

    /// Adds a point correspondence to the computation.
    ///
    /// # Arguments
    ///
    /// * `p1` - The first point in the correspondence.
    /// * `p2` - The second point in the correspondence.
    pub fn add_point_correspondence(&mut self, p1: Point, p2: Point) {
        self.point_correspondences.push(PointPair { p1, p2 });
    }

    /// Adds a line correspondence to the computation.
    ///
    /// # Arguments
    ///
    /// * `l1` - The first line in the correspondence.
    /// * `l2` - The second line in the correspondence.
    pub fn add_line_correspondence(&mut self, l1: Line, l2: Line) {
        self.line_correspondences.push(LinePair { l1, l2 });
    }

    /// Gets the restrictions for the homography computation.
    ///
    /// # Returns
    ///
    /// The restrictions for the homography computation.
    pub fn get_restrictions(&self) -> HomographyRestrictions<f32> {
        let mut restrictions = HomographyRestrictions {
            restrictions: Vec::new(),
        };

        for pair in &self.point_correspondences {
            restrictions.restrictions.push(pair.generate_restriction());
        }

        for pair in &self.line_correspondences {
            restrictions.restrictions.push(pair.generate_restriction());
        }

        restrictions
    }
}

/// A implementation for computing homography restrictions.
///
/// # Example
///
/// ```
/// use homography::{HomographyComputation};
/// use homography::geo::Point;
///
/// let mut hc = HomographyComputation::new();
/// let p1 = Point::new(148., 337.);
/// let p2 = Point::new(131., 516.);
/// let p3 = Point::new(321., 486.);
/// let p4 = Point::new(332., 370.);
///
/// let p1p = Point::new(0., 0.);
/// let p2p = Point::new(0., 60.);
/// let p3p = Point::new(80., 60.);
/// let p4p = Point::new(80., 0.);
///
/// hc.add_point_correspondence(p1, p1p);
/// hc.add_point_correspondence(p2, p2p);
/// hc.add_point_correspondence(p3, p3p);
/// hc.add_point_correspondence(p4, p4p);
///
/// let restrictions = hc.get_restrictions();
///
/// let solution = restrictions.compute();
///
/// println!("Matrix: {}", solution.matrix);
/// println!("Value: {}", solution.value);
///
/// ```
impl<T: Scalar + Copy + num_traits::Zero + RealField> HomographyRestrictions<T> {
    /// Computes the homography solution based on the restrictions.
    ///
    /// # Returns
    ///
    /// The computed homography solution.
    pub fn compute(&self) -> HomographySolution<T> {
        let num_rows = self.restrictions.len() * 2;
        let mut matrix = DMatrix::zeros(num_rows, 9);

        for (i, m) in self.restrictions.iter().enumerate() {
            matrix.set_row(i * 2, &m.row(0));
            matrix.set_row(i * 2 + 1, &m.row(1));
        }

        solve(matrix)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_homography_computation() {
        let hc = HomographyComputation::new();
        assert_eq!(hc.point_correspondences.len(), 0);
        assert_eq!(hc.line_correspondences.len(), 0);
    }

    #[test]
    fn test_add_point_correspondence() {
        let mut hc = HomographyComputation::new();
        let p1 = Point::new(1., 2.);
        let p2 = Point::new(3., 4.);

        hc.add_point_correspondence(p1, p2);
        assert_eq!(hc.point_correspondences.len(), 1);
    }

    #[test]
    fn test_add_line_correspondence() {
        let mut hc = HomographyComputation::new();
        let l1 = Line::new(1., 2., 3.);
        let l2 = Line::new(4., 5., 6.);

        hc.add_line_correspondence(l1, l2);
        assert_eq!(hc.line_correspondences.len(), 1);
    }

    #[test]
    fn test_get_restrictions() {
        let mut hc = HomographyComputation::new();
        let p1 = Point::new(1., 2.);
        let p2 = Point::new(3., 4.);
        let l1 = Line::new(1., 2., 3.);
        let l2 = Line::new(4., 5., 6.);

        hc.add_point_correspondence(p1, p2);
        hc.add_line_correspondence(l1, l2);

        let restrictions = hc.get_restrictions();
        assert_eq!(restrictions.restrictions.len(), 2);
    }

    #[test]
    fn test_compute_homography_solution() {
        let mut hc = HomographyComputation::new();
        let p1 = Point::new(148., 337.);
        let p2 = Point::new(131., 516.);
        let p3 = Point::new(321., 486.);
        let p4 = Point::new(332., 370.);

        let p1p = Point::new(0., 0.);
        let p2p = Point::new(0., 60.);
        let p3p = Point::new(80., 60.);
        let p4p = Point::new(80., 0.);

        let line1 = Line::from_points(&p1, &p2);
        let line1p = Line::from_points(&p1p, &p2p);

        hc.add_point_correspondence(p1, p1p);
        hc.add_point_correspondence(p2, p2p);
        hc.add_point_correspondence(p3, p3p);
        hc.add_point_correspondence(p4, p4p);

        hc.add_line_correspondence(line1, line1p);

        let restrictions = hc.get_restrictions();
        let solution = restrictions.compute();

        assert_eq!(solution.matrix.nrows(), 3);
        assert_eq!(solution.matrix.ncols(), 3);
    }
}
