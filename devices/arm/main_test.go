package main

import (
	"log"
	"github.com/jmoiron/sqlx"
	_ "modernc.org/sqlite"
	"github.com/koeng101/armos/devices/arm/ar3"
	"net/http/httptest"
	"os"
	"strings"
	"testing"
)

var app App

func TestMain(m *testing.M) {
	// Initialize the local sqlite database
	db, err := sqlx.Open("sqlite", ":memory:")
	if err != nil {
		log.Fatalf("Failed to open sqlite database on err: %s", err)
	}
	_, err = db.Exec(Schema)
	if err != nil {
		log.Fatalf("Failed on CreateDatabase with error: %s", err)
	}
	// Initialize ar3 mock arm
	arm := ar3.ConnectMock()
	app = initializeApp(db, arm)

	// Run the rest of our code
	code := m.Run()
	os.Exit(code)
}

func TestPing(t *testing.T) {
	req := httptest.NewRequest("GET", "/api/ping", nil)
	resp := httptest.NewRecorder()
	app.Router.ServeHTTP(resp, req)

	r := `{"message":"Online"}`
	if strings.TrimSpace(resp.Body.String()) != r {
		t.Errorf("Unexpected response. Expected: " + r + "\nGot: " + resp.Body.String())
	}
}
