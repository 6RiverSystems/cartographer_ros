#!/usr/bin/env groovy

def server = Artifactory.server 'sixriver'

parallel(
    failFast: true,
    "amd64": { 
        node('docker && amd64') {
            def customImage = ""
            def scmVars = ""


            stage("amd64 Build and Publish") {
                docker.image('ros:kinetic') {
                    sh '''
                    ./install.sh 
                    mkdir -p artifacts
                    cp -r /opt/cartographer/install_isolated /artifacts 
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
)
