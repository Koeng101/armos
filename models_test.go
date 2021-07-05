package armos

import (
	"github.com/jmoiron/sqlx"
	_ "github.com/mattn/go-sqlite3"
	"log"
	"os"
	"testing"
)

func TestMain(m *testing.M) {
	// Begin SQLite
	db, err = sqlx.Open("sqlite3", ":memory:")
	if err != nil {
		log.Fatalf("Failed to open sqlite in memory: %s", err)
	}

	// Execute our schema in memory
	_, err = db.Exec(Schema)
	if err != nil {
		log.Fatalf("Failed to execute schema: %s", err)
	}

	// Run the rest of our tests
	code := m.Run()

	os.Exit(code)
}
