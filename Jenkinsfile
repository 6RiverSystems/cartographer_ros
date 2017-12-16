#!/usr/bin/env groovy

def server = Artifactory.server 'sixriver'
parallel(
    failFast: true,
    "amd64": {
        node('docker && amd64') {
            def customImage = ""
            def scmVars = ""

            stage("Build Docker Image") {
                scmVars = checkout scm
                sh 'sed -e "s/#DOCKER_IMAGE/ros:kinetic/g" 6river.dockerfile > 6river-amd64.dockerfile'
                customImage = docker.build("gcr.io/plasma-column-128721/ros-builder:amd64", " --file 6river-amd64.dockerfile ." )
            }
            stage("Build and Publish") {
                customImage.inside("-u 0:0 -e GIT_BRANCH=${scmVars.GIT_BRANCH}") {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        ./install.sh 
                        '''
                        // Create the upload spec.
                        def uploadSpec = """{
                            "files": [
                            {
                                "pattern": "${env.WORKSPACE}/artifacts/*.deb",
                                "target": "debian/pool/main/c/cartographer-sixriver/",
                                "props": "deb.distribution=xenial;deb.component=main;deb.architecture=amd64"
                            }
                            ]
                        }"""
                    }
                    // Upload to Artifactory.
                    //server.upload spec: uploadSpec
                }
            }
        }
    },
    "arm64": {
        node('docker && arm64') {
            def customImage = ""
            def scmVars = ""

            stage("Build Docker Image") {
                scmVars = checkout scm
                sh 'sed -e "s/#DOCKER_IMAGE/arm64v8\\/ros:kinetic/g" 6river.dockerfile > 6river-arm64.dockerfile'
                customImage = docker.build("gcr.io/plasma-column-128721/ros-builder:arm64", " --file 6river-arm64.dockerfile ." )
            }
            stage("Build and Publish") {
                customImage.inside("-u 0:0 -e GIT_BRANCH=${scmVars.GIT_BRANCH}") {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        ./install.sh 
                        '''
                        // Create the upload spec.
                        def uploadSpec = """{
                            "files": [
                            {
                                "pattern": "${env.WORKSPACE}/artifacts/*.deb",
                                "target": "debian/pool/main/c/cartographer-sixriver/",
                                "props": "deb.distribution=xenial;deb.component=main;deb.architecture=amd64"
                            }
                            ]
                        }"""
                    }
                    // Upload to Artifactory.
                    server.upload spec: uploadSpec
                }
            }
        }
    }





)
