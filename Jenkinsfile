#!/usr/bin/env groovy

def server = Artifactory.server 'sixriver'

node('docker && amd64') {
	def customImage = ""
	def scmVars = ""

	stage("amd64 Build Docker Image") {
		scmVars = checkout scm
		customImage = docker.build("gcr.io/plasma-column-128721/ros-builder:amd64", " --file 6river.dockerfile ." )
	}
	stage("amd64 Build and Publish") {
		docker.image('ros:kinetic').inside('-u 0:0') {
			sh '''
			./install.sh 
			mkdir -p artifacts
			cp /opt/cartographer/*.deb /artifacts 
			'''
			// Create the upload spec.
			def uploadSpec = """{
				"files": [
				{
			"pattern": "${env.WORKSPACE}/artifacts/*.deb",
				"target": "debian/pool/main/c/cartographer/",
				"props": "deb.distribution=xenial;deb.component=main;deb.architecture=amd64"
				}
				]
			}"""

			// Upload to Artifactory.
			server.upload spec: uploadSpec
		}
	}
}
