// Package main implements the viam-unitree module for the Unitree G1 humanoid robot.
package main

import (
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: base.API, Model: g1BaseModel},
		resource.APIModel{API: camera.API, Model: g1CameraModel},
	)
}
