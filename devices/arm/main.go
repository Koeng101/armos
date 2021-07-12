package main

import (
	"encoding/json"
	"fmt"
	"github.com/jmoiron/sqlx"
	_ "github.com/koeng101/armos/devices/arm/docs" // API docs generated by swaggo/swag
	"github.com/swaggo/swag"
	"log"
	_ "modernc.org/sqlite"
	"net/http"
	"os"
)

/******************************************************************************

				armos arm API

This file contains the armos robotic arm API. The armos server would send
either an API request for the robot to home or an API request for the arm to
move to an `x,y,z,a,b,c` position.

Right now, we only support the AR3 robotic arm, but hopefully will support more
in the future.

This initial portion initializes the API itself.

******************************************************************************/

// App is a struct containing all information about the currently deployed
// application, such as the router and database.
type App struct {
	DB     *sqlx.DB
	Router *http.ServeMux
}

// initalizeApp initializes an App for all endpoints to use.
func initializeApp(db *sqlx.DB) App {
	var app App
	app.DB = db
	app.Router = http.NewServeMux()

	// Basic routes
	app.Router.HandleFunc("/api/ping", app.Ping)
	app.Router.HandleFunc("/swagger.json", app.SwaggerJSON)
	app.Router.HandleFunc("/docs", app.SwaggerDocs)

	app.Router.HandleFunc("/api/home", app.Home)
	app.Router.HandleFunc("/api/calibrate", app.Calibrate)
	app.Router.HandleFunc("/api/move", app.Move)
	app.Router.HandleFunc("/api/status", app.Status)

	return app
}

// @title ArmOS arm API
// @version 0.1
// @description The arm API for ArmOS to interact with a variety of different robotic arms, starting with the AR3. It uses the basic interface of `x,y,z,a,b,c` for control.
// @BasePath /api/
func main() {
	var dbUrl string
	dbUrl = os.Getenv("DATABASE_URL")
	if dbUrl == "" {
		dbUrl = ":memory:"
	}
	db, err := sqlx.Open("sqlite", dbUrl)
	if err != nil {
		log.Fatalf("Failed to connect to database with error: %s", err)
	}
	//_ = CreateDatabase(db)
	app := initializeApp(db)

	// Serve application
	s := &http.Server{
		Addr:    ":8080",
		Handler: app.Router,
	}
	log.Fatal(s.ListenAndServe())
}

/******************************************************************************

				armos arm models

******************************************************************************/

// Coordinate is a struct containing the ABCXYZ coordinates of a given robotic
// arm position.
type Coordinate struct {
	A float64 `json:"a"`
	B float64 `json:"b"`
	C float64 `json:"c"`
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// Response is the response given to a status query on the robotic API.
type Response struct {
	Id     int    `db:"id" json:"id"`
	Status string `db:"status" json:"status"`
	Start  int    `db:"start" json:"start"`
	End    int    `db:"end" json:"end"`

	From     Coordinate `json:"from"`
	To       Coordinate `json:"to"`
	Backlash Coordinate `json:"backlash"`
}

/******************************************************************************

                                armos arm routes

ArmOS's robotic arm API has 4 core endpoints: /home, /calibrate, /move, and
/status.

1. /home homes the robot to where it can be safely turned off.
2. /calibrate calibrates the robotic arm to its limit switches.
3. /move moves the robotic arm to a certain abcxyz location.
4. /status gives the status of a /home, /calibrate, or /move command.

******************************************************************************/

// Ping is a simple route for verifying that the service is online.
// @Summary A pingable endpoint
// @Tags dev
// @Produce json
// @Success 200 {object} map[string]string
// @Router /ping [get]
func (app *App) Ping(w http.ResponseWriter, r *http.Request) {
	encoder := json.NewEncoder(w)
	_ = encoder.Encode(map[string]string{"message": "Online"})
}

// SwaggerJSON provides the swagger docs for this api in JSON format.
func (app *App) SwaggerJSON(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Access-Control-Allow-Origin", "*")
	doc, _ := swag.ReadDoc()
	_, _ = w.Write([]byte(doc))
}

// SwaggerDocs provides a human-friendly swagger ui interface.
func (app *App) SwaggerDocs(w http.ResponseWriter, r *http.Request) {
	// https://stackoverflow.com/questions/55733609/display-swagger-ui-on-flask-without-any-hookups
	doc, _ := swag.ReadDoc()
	swaggerDoc := fmt.Sprintf(`<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="X-UA-Compatible" content="ie=edge">
    <script src="//unpkg.com/swagger-ui-dist@3/swagger-ui-standalone-preset.js"></script>
    <!-- <script src="https://cdnjs.cloudflare.com/ajax/libs/swagger-ui/3.22.1/swagger-ui-standalone-preset.js"></script> -->
    <script src="//unpkg.com/swagger-ui-dist@3/swagger-ui-bundle.js"></script>
    <!-- <script src="https://cdnjs.cloudflare.com/ajax/libs/swagger-ui/3.22.1/swagger-ui-bundle.js"></script> -->
    <link rel="stylesheet" href="//unpkg.com/swagger-ui-dist@3/swagger-ui.css" />
    <!-- <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/swagger-ui/3.22.1/swagger-ui.css" /> -->
    <title>Swagger</title>
</head>
<body>
    <div id="swagger-ui"></div>
    <script>
        window.onload = function() {
          SwaggerUIBundle({
	    spec: %s,
            dom_id: '#swagger-ui',
            presets: [
              SwaggerUIBundle.presets.apis,
              SwaggerUIStandalonePreset
            ],
            layout: "StandaloneLayout"
          })
        }
    </script>
</body>
</html>`, doc)
	_, _ = w.Write([]byte(swaggerDoc))
}

// Home homes the robot. This is useful for when the robotic arm will be
// turned off - otherwise, it will collapse under its own weight.
// @Summary Home the arm
// @Tags robot
// @Description Homes the robot. Useful for when cutting power to the robotic arm.
// @Produce json
// @Success 200 {object} Response
// @Failure 400 {object} Response
// @Router /home [get]
func (app *App) Home(w http.ResponseWriter, r *http.Request) {
	_ = json.NewEncoder(w).Encode(Response{})
}

// Calibrate calibrates the robot to its limit switches.
// @Summary Calibrate the arm
// @Tags robot
// @Description Calibrates the robot. Should be done occasionally to affirm the robot is where we think it should be.
// @Produce json
// @Success 200 {object} Response
// @Failure 400 {object} Response
// @Router /calibrate [get]
func (app *App) Calibrate(w http.ResponseWriter, r *http.Request) {
	_ = json.NewEncoder(w).Encode(Response{})
}

// Move moves the robot to a certain abcxyz position
// @Summary Move the arm
// @Tags robot
// @Description Moves the robot to a certain position.
// @Accept json
// @Produce json
// @Param move body Coordinate true "abcxyz coordinate"
// @Success 200 {object} Response
// @Failure 400 {object} Response
// @Router /move [post]
func (app *App) Move(w http.ResponseWriter, r *http.Request) {
	_ = json.NewEncoder(w).Encode(Response{})
}

// Status gives the status of a move command.
// @Summary Status of a command
// @Tags robot
// @Description Gives the status of a home, calibrate, or move command.
// @Produce json
// @Param id path int true "Command ID"
// @Success 200 {object} Response
// @Failure 400 {object} Response
// @Router /status [get]
func (app *App) Status(w http.ResponseWriter, r *http.Request) {
	_ = json.NewEncoder(w).Encode(Response{})
}
