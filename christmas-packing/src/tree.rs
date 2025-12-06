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

const SCALING_FACTOR: f64 = 10000.0;
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

#[derive(Debug, Clone)]
pub struct ChristmasTree {
    points: Vec<Point>,
    rotation: f64,
}

impl ChristmasTree {
    pub fn new(deg: f64) -> ChristmasTree {
        let points = rotate(&scale(&FULL_TREE), deg);
        Self {
            points,
            rotation: deg,
        }
    }

    pub fn points(&self) -> &[Point] {
        &self.points
    }

    pub fn convex_decomp(&self) -> [Vec<Point>; 4] {
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
        [top, mid, low, trunk]
    }

    pub fn bounds(&self) -> PieceBounds {
        let x_min = self
            .points
            .iter()
            .map(|p| p.x)
            .fold(f64::INFINITY, f64::min);
        let x_max = self
            .points
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max);
        let y_min = self
            .points
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);
        let y_max = self
            .points
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
