# Underwater Kite Turbine Simulation and Control

Description...

## Installation

...

```bash
git clone [https://github.com/rama0422/kite-turbine-control.git]
cd repo
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

## Usage

```python
# From repo folder run e.g.:
python -m src.experiments.sim_system_test
```

## Project Structure

```
kite-turbine-control/ 
├── Data
│ └── MinestoKitePlots.ipynb              # ploting tool for kite data
├── src
│ ├── controllers
│ │ └── og_controller.py                  # initial controller
│ ├── experiments
│ │ ├── controller_real_data_test.py      # testing and comparsion of controllers with data
│ │ ├── plot_coefmaps.py                  # ploting for all coefficient maps
│ │ ├── plot_path.py                      # ploting for of the predetermined fig-8 path
│ │ ├── sensors_model_test.py             # test noise configs on fabricated measurment data
│ │ ├── sim_kite_test.py                  # kite only test
│ │ ├── sim_system_test.py                # kite + turbine + controller test
│ │ └── sim_turbine_test.py               # turbine only test
│ ├── simulation
│ │ ├── full_system_model.py
│ │ ├── functions.py                      # functions used in the models
│ │ ├── kite_model.py
│ │ ├── sensors_model.py
│ │ └── turbine_model.py
│ └── utility
│   └── configs.py                        # params for all part of the system
├── requirements.txt                      # dependencies
└── README.md                             # documentation
```

## Contributing

...

## License

...
