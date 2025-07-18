mod data {
    use core::f64;
    use rand::Rng;
    use mint::{Quaternion};

    pub struct DataState {
        sigma: f64,
        clock: f64,
        quat: Quaternion<f64>,
    }

    impl DataState {
        pub fn new() -> Self {
            Self {
                sigma: 1.0,//: f64::INFINITY,
                clock: 0.0,
                quat: mint::Quaternion { s: 1.0, v: mint::Vector3 { x: 0.0, y: 0.0, z: 0.0 } },
            }
        }
    }

    xdevs::component!(
        ident = Data,
        output = {
            data<mint::Quaternion<f64>>,
        },
        state = DataState,
    );

    impl xdevs::Atomic for Data {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            let w: f64 = rand::thread_rng().gen_range(-1.0..=1.0);
            let x: f64 = rand::thread_rng().gen_range(-1.0..=1.0);
            let y: f64 = rand::thread_rng().gen_range(-1.0..=1.0);
            let z: f64 = rand::thread_rng().gen_range(-1.0..=1.0);
            // Normalize the quaternion to be a valid quaternion 
            let norm = (w*w + x*x + y*y + z*z).sqrt();
            let quat = mint::Quaternion {
                s: w / norm,
                v: mint::Vector3 { x: x / norm, y: y / norm, z: z / norm },
            };
            state.quat = quat;

            println!("Quaternion generated : {:?} at t={} ", state.quat, state.clock);
            state.sigma = 3.;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            //put the value of orientation on output
            let _ = output.data.add_value(state.quat);
            //println!("Data sent !")
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
        }
    }
}



mod calculator {
    use core::f64;

    #[derive(Default)]
    pub struct CalculatorState {
        sigma: f64,
        clock: f64,
        orientation: (f64, f64, f64), 
    }

    impl CalculatorState {
        pub fn new() -> Self {
            Self {
                sigma: f64::INFINITY,
                clock: 0.0,
                orientation: (0.0,0.0,0.0), //at the beginning it's the orientation of reference or 0,0,0
            }
        }
    }

    //Quaternion {s: w, // scalare
    //            v: Vector3 {x: x, y: y, z: z, }}

    xdevs::component!(
        ident = Calculator,
        input= {
            data<mint::Quaternion<f64>>,
        },
        output = {
            position<(f64, f64, f64)>,
        },
        state = CalculatorState,
    );

    impl xdevs::Atomic for Calculator {
        fn delta_int(state: &mut Self::State) {
            //wait to have extern values from sensor
            state.clock += state.sigma;
            println!("Waiting data t={} ", state.clock);
            state.sigma = f64::INFINITY;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            //put the value of orientation on output
            let _ = output.position.add_value(state.orientation);
            println!("Orientation sent !")
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
            //quaternion register : 8bytes -> wLSB, wMSB, xLSB, xMSB, yLSB, yMSB, zLSB, zMSB
            /*quaternion form : Quaternion {s: w, // scalare
                                            v: Vector3 {x: x, y: y, z: z, }}*/
            
            let values = input.data.get_values();
            if !values.is_empty() {
                //quat is not empty so we take the values 
                let quat = values[0]; // first element
                let w = quat.s;
                let x = quat.v.x;
                let y = quat.v.y;
                let z = quat.v.z;
                println!("Quaternion : w={}, x={}, y={}, z={}", w, x, y, z);

                //calculate euler angle thanks to quaternion -> calculate roll, pitch, yaw
                //roll : rotation around the longitudinal axis (nose to tail)
                //pitch : rotation around the lateral axis (wing to wing)
                //yaw : rotation around the vertical axis (top to bottom)
                let roll = (2.0_f64*(w*x + y*z)).atan2(1.0_f64 - 2.0_f64*(x*x + y*y)); //_f64 to precise what type we are using
                let pitch = (2.0_f64*(w*y - z*x)).asin();
                let yaw = (2.0_f64*(w*z + x*y)).atan2(1.0 - 2.0*(y*y + z*z));
                state.orientation = (pitch, roll, yaw);
                println!("Orientation : {:?}", state.orientation);
            }
            state.sigma = 3.;
        }
    }
}


xdevs::component!(
    ident = DataCalculator,
    output = {
        position<(f64, f64, f64)>,
    },
    components = {
        data: data::Data,
        calculator: calculator::Calculator,
    },
    couplings = {
        data.data -> calculator.data,
    }
);

fn main() {
    log::info!("Hello, world!");

    let data= data::Data::new(data::DataState::new());
    let calculator = calculator::Calculator::new(calculator::CalculatorState::new());

    let calculatoranalyser = DataCalculator::new(data, calculator);

    log::info!("Starting simulation!");
    let mut simulator = xdevs::simulator::Simulator::new(calculatoranalyser);
    let config = xdevs::simulator::Config::new(0.0, 60.0, 1.0, None);
    simulator.simulate_vt(&config);
    /*simulator.simulate_rt(
        &config,
        xdevs::simulator::std::sleep(&config),
        |_| {}, //hardware
    );*/
}
