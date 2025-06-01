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
    name TEXT NOT NULL UNIQUE,
    type TEXT CHECK(type IN ('material', 'operation')) NOT NULL,
    loc_x REAL, loc_y REAL, loc_z REAL,
    loc_ox REAL, loc_oy REAL, loc_oz REAL, loc_ow REAL
);


CREATE TABLE IF NOT EXISTS materials (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    type INTEGER NOT NULL CHECK(type IN (1, 2)),
    enabled INTEGER DEFAULT 1 CHECK(enabled IN (0, 1)),
    station_id INTEGER,
    is_liquid INTEGER NOT NULL DEFAULT 0 CHECK(is_liquid IN (0, 1)),
    is_topping INTEGER NOT NULL DEFAULT 0 CHECK(is_topping IN (0, 1)),
    topping_position TEXT CHECK(topping_position IN ('bottom', 'top') OR topping_position IS NULL),
    loc_x REAL, loc_y REAL, loc_z REAL,
    loc_ox REAL, loc_oy REAL, loc_oz REAL, loc_ow REAL,
    FOREIGN KEY (station_id) REFERENCES stations(id)
);


DELETE FROM orders;
DELETE FROM workflow_steps;
DELETE FROM materials;
DELETE FROM stations;
DELETE FROM recipes;


INSERT INTO recipes (id, name, description, liquid_ingredients, default_toppings) VALUES
(1, 'Pearl Milk Tea', 'Black tea with milk and pearls', 'tea,milk', 'pearl'),
(2, 'Grass Jelly Milk Tea', 'Smooth tea with grass jelly', 'tea,milk', 'coco');


INSERT INTO stations (id, name, type, loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow) VALUES
(1, 'cup_station',      'operation',  0.30,  0.00, 0.90, 0.0, 0.0, 0.0, 1.0),
(2, 'milk_station',     'material',   0.45,  0.25, 1.05, 0.0, 0.0, 0.0, 1.0),
(3, 'tea_station',      'material',   0.45, -0.25, 1.05, 0.0, 0.0, 0.0, 1.0),
(4, 'pearl_station',    'material',   0.60,  0.00, 1.00, 0.0, 0.0, 0.0, 1.0),
(5, 'coco_station',     'material',   0.60,  0.30, 1.00, 0.0, 0.0, 0.0, 1.0),
(6, 'discard_station',  'operation',  0.75,  0.00, 0.90, 0.0, 0.0, 0.0, 1.0),
(7, 'pouring_spot',     'operation',  0.50,  0.00, 1.10, 0.0, 0.0, 0.0, 1.0),
(8, 'home_station',     'operation',  0.55,  0.40, 1.10, 0.0, 0.0, 0.0, 1.0);



INSERT INTO materials (id, name, type, station_id, is_liquid, is_topping, topping_position, loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow) VALUES
(1, 'small_cup', 2, 1, 0, 0, NULL,0.30, 0.00, 0.90, 0.0, 0.0, 0.0, 1.0),
(2, 'milk',  1, 2, 1, 0, NULL,    0.45,  0.25, 1.05, 0.0, 0.0, 0.0, 1.0),
(3, 'tea',   1, 3, 1, 0, NULL,    0.45, -0.25, 1.05, 0.0, 0.0, 0.0, 1.0),
(4, 'pearl', 2, 4, 0, 1, 'bottom',0.60,  0.00, 1.00, 0.0, 0.0, 0.0, 1.0),
(5, 'coco',  2, 5, 0, 1, 'top',   0.60,  0.30, 1.00, 0.0, 0.0, 0.0, 1.0),
(6, 'rubbish_bin', 2, 6, 0, 0, NULL, 0.75, 0.00, 0.90, 0.0, 0.0, 0.0, 1.0),
(7, 'pour_spot', 2, 7, 0, 0, NULL,0.50, 0.00, 1.10, 0.0, 0.0, 0.0, 1.0);

