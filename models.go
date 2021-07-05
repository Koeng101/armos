package armos

import (
	"fmt"
)

var Schema = `
-- Start schema --

CREATE TABLE IF NOT EXISTS object (
	uuid TEXT PRIMARY KEY,
	name TEXT,
	address TEXT NOT NULL,
	type TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS transformation (
	parent TEXT NOT NULL REFERENCES object(uuid),
	object TEXT NOT NULL REFERENCES object(uuid),
	x REAL NOT NULL,
	y REAL NOT NULL,
	z REAL NOT NULL,
	qw REAL NOT NULL,
	qx REAL NOT NULL,
	qy REAL NOT NULL,
	qz REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS queue (
	createdat INT NOT NULL,
	startedat INT NOT NULL,
	completedat INT NOT NULL,

	uuid TEXT PRIMARY KEY,
	status TEXT NOT NULL CHECK (status IN ('queued', 'working', 'complete')),
	command JSON NOT NULL -- Figure this out later
)

CREATE TABLE IF NOT EXISTS dependencies (
	task TEXT NOT NULL REFERENCES queue(uuid) ON DELETE CASCADE,
	dependson TEXT NOT NULL REFERENCES queue(uuid) ON DELETE CASCADE
);
`
