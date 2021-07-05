package armos

import (
	"fmt"
	"github.com/jmoiron/sqlx"
	_ "github.com/mattn/go-sqlite3"
	"log"
	"os"
	"testing"
)

var db *sqlx.DB

func TestMain(m *testing.M) {
	// Begin SQLite
	var err error
	db, err = sqlx.Open("sqlite3", ":memory:")
	if err != nil {
		log.Fatalf("Failed to open sqlite in memory: %s", err)
	}

	// Execute our schema in memory
	_, err = db.Exec(Schema)
	if err != nil {
		log.Fatalf("Failed to execute schema: %s", err)
	}

	// Begin inserting some basic data
	tx, err := db.Beginx()
	if err != nil {
		log.Fatalf("Initialize transaction failed in TestMain. Got error: %s", err)
	}
	objects := []Object{{"0", "root", "https://192.168.1.16", "root"}, {"1", "ar3", "https://192.168.1.17", "ar3"}, {"2", "ar3_end_effector", "https://192.168.1.18", "ar3_end_effector"}, {"3", "opentrons", "https://192.168.1.19", "opentrons"}}

	for _, object := range objects {
		err = object.Insert(tx)
		if err != nil {
			log.Fatalf("Failed to insert %s in TestMain. Got error: %s", object.Name, err)
		}
	}

	// Insert some basic transform data
	transformations := []Transformation{{"0", "1", 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {"1", "2", 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0}, {"0", "3", 30.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0}}

	for _, transformation := range transformations {
		err = transformation.Insert(tx)
		if err != nil {
			log.Fatalf("Failed to insert transformation in TestMain. Got error: %s", err)
		}
	}

	// Commit
	err = tx.Commit()
	if err != nil {
		log.Fatalf("Failed to commit basic data in TestMain. Got error: %s", err)
	}

	// Run the rest of our tests
	code := m.Run()

	os.Exit(code)
}

func ExampleTransformationsBetween() {
	tx := db.MustBegin()
	// Transform between ar3_end_effector and opentrons
	transformations, _ := TransformationsBetween(tx, "2", "3")
	_ = tx.Rollback()

	fmt.Println(transformations)
	// Output: [{1 2 0 0 10 0 0 0 0} {0 1 10 10 0 0 0 0 0} {0 3 30 30 0 0 0 0 0}]
}
