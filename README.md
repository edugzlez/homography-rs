# Homography ~ Rust

 [![crates.io](https://img.shields.io/crates/v/homography.svg)](https://crates.io/crates/homography) [![docs.rs](https://docs.rs/homography/badge.svg)](https://docs.rs/electosim) [![codecov](https://codecov.io/gh/edugzlez/homography-rs/branch/master/graph/badge.svg?token=1KGDZPWBRI)](https://codecov.io/gh/edugzlez/homography-rs)

Homography is a Rust library for computing the homography matrix between two planes in a 2D space. It is based on the Direct Linear Transformation (DLT) algorithm and it is designed to be used in computer vision applications.

The project is under development and will feature new updates and improvements.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
homography = "0.1.0"
```

or add it directly from crates.io:

```sh
cargo add homography
```

## Usage

```rust
use homography::{HomographyComputation};
use homography::geo::{Point, Line};

fn main() {
    // Create a new instance of HomographyComputation
    let hc = HomographyComputation::new();

    // Define points
    let p1 = Point::new(148., 337.);
    let p2 = Point::new(131., 516.);
    let p3 = Point::new(321., 486.);
    let p4 = Point::new(332., 370.);
    let p1p = Point::new(0., 0.);
    let p2p = Point::new(0., 60.);
    let p3p = Point::new(80., 60.);
    let p4p = Point::new(80., 0.);

    // Define lines
    let line1 = Line::from_points(&p1, &p2);
    let line1p = Line::from_points(&p1p, &p2p);

    // Add point and line correspondences
    hc.add_point_correspondence(p1, p1p);
    hc.add_point_correspondence(p2, p2p);
    hc.add_point_correspondence(p3, p3p);
    hc.add_point_correspondence(p4, p4p);
    hc.add_line_correspondence(line1, line1p);

    // Get restrictions and compute solution
    let restrictions = hc.get_restrictions();
    let solution = restrictions.compute();

    // Print the solution
    println!("Matrix: {}", solution.matrix);
    println!("Value: {}", solution.value);
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
