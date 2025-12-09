use nfp::Point;

pub const FULL_TREE: [Point; 15] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.125, y: 0.5 },
    Point { x: -0.0625, y: 0.5 },
    Point { x: -0.2, y: 0.25 },
    Point { x: -0.1, y: 0.25 },
    Point { x: -0.35, y: 0.0 },
    Point { x: -0.075, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.075, y: 0.0 },
    Point { x: 0.35, y: 0.0 },
    Point { x: 0.1, y: 0.25 },
    Point { x: 0.2, y: 0.25 },
    Point { x: 0.0625, y: 0.5 },
    Point { x: 0.125, y: 0.5 },
];

pub const SIMPLE_TREE: [Point; 7] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.35, y: 0.0 },
    Point { x: -0.075, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.075, y: 0.0 },
    Point { x: 0.35, y: 0.0 },
];

pub const CONVEX_HULL: [Point; 5] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.35, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.35, y: 0.0 },
];

pub const SCALING_FACTOR: f64 = 100.0;
const TOP_TIER: [Point; 3] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.125, y: 0.5 },
    Point { x: 0.125, y: 0.5 },
];

const MID_TIER: [Point; 4] = [
    Point { x: -0.0625, y: 0.5 },
    Point { x: -0.2, y: 0.25 },
    Point { x: 0.2, y: 0.25 },
    Point { x: 0.0625, y: 0.5 },
];

const LOW_TIER: [Point; 4] = [
    Point { x: -0.1, y: 0.25 },
    Point { x: -0.35, y: 0.0 },
    Point { x: 0.35, y: 0.0 },
    Point { x: 0.1, y: 0.25 },
];

const TRUNK: [Point; 4] = [
    Point { x: -0.075, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.075, y: 0.0 },
];

pub fn scale(poly: &[Point]) -> Vec<Point> {
    poly.iter()
        .map(|p| Point::new(p.x * SCALING_FACTOR, p.y * SCALING_FACTOR))
        .collect()
}

pub fn rotate(points: &[Point], deg: f64) -> Vec<Point> {
    points
        .iter()
        .map(|p| {
            let new_x = deg.to_radians().cos() * p.x - deg.to_radians().sin() * p.y;
            let new_y = deg.to_radians().sin() * p.x + deg.to_radians().cos() * p.y;
            Point::new(new_x, new_y)
        })
        .collect()
}

pub trait Tree: Clone {
    fn new(deg: f64) -> Self;
    fn points(&self) -> &[Point];
    fn translate(&self, t: Point) -> Self;
    fn convex_decomp(&self) -> Vec<Vec<Point>>;

    fn bounds(&self) -> PieceBounds {
        let x_min = self
            .points()
            .iter()
            .map(|p| p.x)
            .fold(f64::INFINITY, f64::min);
        let x_max = self
            .points()
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max);
        let y_min = self
            .points()
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);
        let y_max = self
            .points()
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max);
        PieceBounds {
            l_min: x_min.abs(),
            l_max: x_max.abs(),
            h_min: y_min.abs(),
            h_max: y_max.abs(),
            width: x_min.abs() + x_max.abs(),
            height: y_min.abs() + y_max.abs(),
        }
    }
    fn rotation(&self) -> f64;
}

#[derive(Debug, Clone)]
pub struct ChristmasTree {
    points: Vec<Point>,
    pub rotation: f64,
}

impl Tree for ChristmasTree {
    fn new(deg: f64) -> ChristmasTree {
        let points = rotate(&scale(&FULL_TREE), deg);
        Self {
            points,
            rotation: deg,
        }
    }

    fn rotation(&self) -> f64 {
        self.rotation
    }

    fn points(&self) -> &[Point] {
        &self.points
    }

    fn translate(&self, t: Point) -> Self {
        let points = self
            .points()
            .iter()
            .map(|p| Point::new(p.x + t.x, p.y + t.y))
            .collect();
        Self {
            points,
            rotation: self.rotation,
        }
    }

    fn convex_decomp(&self) -> Vec<Vec<Point>> {
        let top = vec![self.points[0], self.points[1], self.points[14]];
        let mid = vec![
            self.points[2],
            self.points[3],
            self.points[12],
            self.points[13],
        ];
        let low = vec![
            self.points[4],
            self.points[5],
            self.points[10],
            self.points[11],
        ];
        let trunk = vec![
            self.points[6],
            self.points[7],
            self.points[8],
            self.points[9],
        ];
        vec![top, mid, low, trunk]
    }
}

#[derive(Debug, Clone)]
pub struct SimpleTree {
    points: Vec<Point>,
    pub rotation: f64,
}

impl Tree for SimpleTree {
    fn new(deg: f64) -> Self {
        let points = rotate(&scale(&SIMPLE_TREE), deg);
        Self {
            points,
            rotation: deg,
        }
    }

    fn rotation(&self) -> f64 {
        self.rotation
    }

    fn points(&self) -> &[Point] {
        &self.points
    }

    fn translate(&self, t: Point) -> Self {
        let points = self
            .points
            .iter()
            .map(|p| Point::new(p.x + t.x, p.y + t.y))
            .collect();
        Self {
            points,
            rotation: self.rotation,
        }
    }

    fn convex_decomp(&self) -> Vec<Vec<Point>> {
        let top = vec![self.points[0], self.points[1], self.points[6]];
        let trunk = vec![
            self.points[2],
            self.points[3],
            self.points[4],
            self.points[5],
        ];
        vec![top, trunk]
    }
}

#[derive(Debug, Clone)]
pub struct ConvexHull {
    points: Vec<Point>,
    pub rotation: f64,
}

impl Tree for ConvexHull {
    fn new(deg: f64) -> Self {
        let points = rotate(&scale(&CONVEX_HULL), deg);
        Self {
            points,
            rotation: deg,
        }
    }

    fn rotation(&self) -> f64 {
        self.rotation
    }

    fn points(&self) -> &[Point] {
        &self.points
    }

    fn translate(&self, t: Point) -> Self {
        let points = self
            .points
            .iter()
            .map(|p| Point::new(p.x + t.x, p.y + t.y))
            .collect();
        Self {
            points,
            rotation: self.rotation,
        }
    }

    fn convex_decomp(&self) -> Vec<Vec<Point>> {
        vec![self.points.to_vec()]
    }
}

#[derive(Debug, Clone)]
pub struct PieceBounds {
    pub l_min: f64,
    pub l_max: f64,
    pub h_min: f64,
    pub h_max: f64,
    pub width: f64,
    pub height: f64,
}

impl PieceBounds {
    pub fn max_diameter(&self) -> f64 {
        self.width.max(self.height)
    }
}
