
DROP TABLE IF EXISTS recipes;
DROP TABLE IF EXISTS orders;
DROP TABLE IF EXISTS workflow_steps;
DROP TABLE IF EXISTS stations;
DROP TABLE IF EXISTS materials;


CREATE TABLE IF NOT EXISTS recipes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    description TEXT,
    liquid_ingredients TEXT,
    default_toppings TEXT
);


CREATE TABLE IF NOT EXISTS orders (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recipe_id INTEGER NOT NULL,
    sugar_level TEXT,
    ice_level TEXT,
    toppings TEXT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    status TEXT NOT NULL DEFAULT 'unprepared',
    FOREIGN KEY (recipe_id) REFERENCES recipes(id)
);


CREATE TABLE IF NOT EXISTS workflow_steps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    order_id INTEGER NOT NULL,
    recipe_id INTEGER NOT NULL,
    step_order INTEGER NOT NULL,
    action TEXT NOT NULL,
    material TEXT,
    station_id INTEGER,
    FOREIGN KEY (order_id) REFERENCES orders(id),
    FOREIGN KEY (recipe_id) REFERENCES recipes(id),
    FOREIGN KEY (station_id) REFERENCES stations(id)
);


CREATE TABLE IF NOT EXISTS stations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    type TEXT CHECK(type IN ('material', 'operation')) NOT NULL,
    loc_x REAL, loc_y REAL, loc_z REAL,
    loc_ox REAL, loc_oy REAL, loc_oz REAL, loc_ow REAL
);


CREATE TABLE IF NOT EXISTS materials (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    type INTEGER NOT NULL CHECK(type IN (1, 2)),
    enabled INTEGER DEFAULT 1 CHECK(enabled IN (0, 1)),
    station_id INTEGER,
    is_liquid INTEGER NOT NULL DEFAULT 0 CHECK(is_liquid IN (0, 1)),
    is_topping INTEGER NOT NULL DEFAULT 0 CHECK(is_topping IN (0, 1)),
    topping_position TEXT CHECK(topping_position IN ('bottom','top') OR topping_position IS NULL),
    FOREIGN KEY (station_id) REFERENCES stations(id)
);


DELETE FROM workflow_steps;
DELETE FROM orders;
DELETE FROM materials;
DELETE FROM stations;
DELETE FROM recipes;


INSERT INTO recipes (id, name, description, liquid_ingredients, default_toppings) VALUES
(1, 'Mango Green Tea', 'Green tea with mango chunks', 'green_tea', 'mango'),
(2, 'Strawberry Jasmine Tea','Jasmine tea with strawberry', 'jasmine_tea', 'strawberry');


INSERT INTO stations (id, name, type, loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow) VALUES
(1, 'cup_station',      'operation',  0.50,  0.00, 0.00, 0.0, 0.0, 0.0, 1.0),
(2, 'green_station',    'material',   1.00,  0.30, 0.00, 0.0, 0.0, 0.0, 1.0),
(3, 'jasmine_station',  'material',   1.20, -0.30, 0.00, 0.0, 0.0, 0.0, 1.0),
(4, 'mango_station',    'material',   1.50,  0.10, 0.00, 0.0, 0.0, 0.0, 1.0),
(5, 'strawberry_station','material',   1.70,  0.30, 0.00, 0.0, 0.0, 0.0, 1.0),
(6, 'discard_station',  'operation',  2.00,  0.00, 0.00, 0.0, 0.0, 0.0, 1.0),
(7, 'pouring_spot',     'operation',  1.00,  0.00, 0.00, 0.0, 0.0, 0.0, 1.0);

INSERT INTO materials (id, name, type, station_id, is_liquid, is_topping, topping_position) VALUES
(1, 'green_tea',    1, 2, 1, 0, NULL),
(2, 'jasmine_tea',  1, 3, 1, 0, NULL),
(3, 'mango',        2, 4, 0, 1, 'bottom'),
(4, 'strawberry',   2, 5, 0, 1, 'bottom');

