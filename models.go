package armos

import (
	"github.com/jmoiron/sqlx"
)

var Schema = `

CREATE TABLE IF NOT EXISTS object (
	uuid TEXT PRIMARY KEY,
	name TEXT NOT NULL,
	address TEXT UNIQUE NOT NULL,
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
);

CREATE TABLE IF NOT EXISTS dependencies (
	task TEXT NOT NULL REFERENCES queue(uuid) ON DELETE CASCADE,
	dependson TEXT NOT NULL REFERENCES queue(uuid) ON DELETE CASCADE
);
`

// Schema stuff

type Object struct {
	Uuid       string `db:"uuid"`
	Name       string `db:"name"`
	Address    string `db:"address"`
	ObjectType string `db:"type"`
}

type Transformation struct {
	Parent string  `db:"parent"`
	Object string  `db:"object"`
	X      float64 `db:"x"`
	Y      float64 `db:"y"`
	Z      float64 `db:"z"`
	Qw     float64 `db:"qw"`
	Qx     float64 `db:"qx"`
	Qy     float64 `db:"qy"`
	Qz     float64 `db:"qz"`
}

func (o *Object) Insert(tx *sqlx.Tx) error {
	_, err := tx.Exec("INSERT INTO object (uuid, name, address, type) VALUES (?, ?, ?, ?)", o.Uuid, o.Name, o.Address, o.ObjectType)
	if err != nil {
		return err
	}
	return nil
}

func (t *Transformation) Insert(tx *sqlx.Tx) error {
	_, err := tx.Exec("INSERT INTO transformation (parent, object, x, y, z, qw, qx, qy, qz) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)", t.Parent, t.Object, t.X, t.Y, t.Z, t.Qw, t.Qx, t.Qy, t.Qz)
	if err != nil {
		return err
	}
	return nil
}

// Transforms between two target devices given their UUIDs
func TransformationsBetween(tx *sqlx.Tx, source string, target string) ([]Transformation, error) {
	// Use a recursive CTE to path up towards root node
	sqlCte := `WITH RECURSIVE traverse(parentT, obj, xT, yT, zT, qwT, qxT, qyT, qzT) AS (
  SELECT ?, "test", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  UNION
  SELECT parent, parentT, x, y, z, qw, qx, qy, qz FROM transformation JOIN traverse ON object = parentT
) SELECT parentT AS parent, obj AS object, xT AS x, yT AS y, zT AS z, qwT AS qw, qxT AS qx, qyT AS qy, qzT AS qz FROM traverse;`

	// Get path from source to root
	var toRoot []Transformation
	err := tx.Select(&toRoot, sqlCte, source)
	if err != nil {
		return []Transformation{}, err
	}
	toRoot = toRoot[1:]

	// Get path from target to root
	var fromRoot []Transformation
	err = tx.Select(&fromRoot, sqlCte, target)
	if err != nil {
		return []Transformation{}, err
	}
	fromRoot = fromRoot[1:]

	// Reverse fromRoot
	for i, j := 0, len(fromRoot)-1; i < j; i, j = i+1, j-1 {
		fromRoot[i], fromRoot[j] = fromRoot[j], fromRoot[i]
	}

	// Append them together and return
	return append(toRoot, fromRoot...), nil
}
