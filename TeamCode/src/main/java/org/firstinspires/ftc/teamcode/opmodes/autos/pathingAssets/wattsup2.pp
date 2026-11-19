{
  "version": "2.0.0",
  "timestamp": "2026-11-19T19:53:52.918Z",
  "startPoint": {
    "x": 54.5,
    "y": 8,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mwj0ewa8-p3iolw",
      "color": "#ffc516",
      "name": "goGrabGarden",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 8,
        "y": 13
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "startDeg": 90,
        "endDeg": 180,
        "degrees": -90,
        "reverse": false,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "id": "line-mwj0gt0j-1osaj1",
      "color": "#f28e2b",
      "name": "grabGarden",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 8,
        "y": 8
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": -90,
        "startDeg": -90,
        "endDeg": 90,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "id": "line-mwnyqe6s-ow0mhd",
      "color": "#e15759",
      "name": "goGrabFlowerOne",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 17.92153466507858,
        "y": 46.84221572926609
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    },
    {
      "id": "line-mwnyr3z4-obw36b",
      "color": "#9c755f",
      "name": "grabFlowerOne",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 8.545142734614373,
        "y": 47.02587480292641
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    },
    {
      "id": "line-mwj0hanf-0rd3w3",
      "color": "#edc948",
      "name": "shootLeft",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 46.7,
        "y": 125
      },
      "controlPoints": [
        {
          "x": 33.47159250480237,
          "y": 65.56876698736004
        },
        {
          "x": 22.57757917660808,
          "y": 102.05041903591984
        }
      ],
      "heading": {
        "type": "linear",
        "reverse": true,
        "degrees": 0,
        "startDeg": -90,
        "endDeg": 90,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "id": "line-mwj0i0ei-3rzo3w",
      "color": "#e15759",
      "name": "grabFlower",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 46.7,
        "y": 130
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": 90,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "id": "line-mwj0iive-4e0rvh",
      "color": "#9c755f",
      "name": "park",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 10.689068293403647,
        "y": 114.5117177205669
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "reverse": true,
        "degrees": 0,
        "startDeg": 90,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    }
  ],
  "shapes": [
    {
      "id": "biobuzz-red-hive",
      "name": "Red HIVE",
      "vertices": [
        {
          "x": 46,
          "y": 50
        },
        {
          "x": 48.6,
          "y": 50
        },
        {
          "x": 48.6,
          "y": 91.4
        },
        {
          "x": 46,
          "y": 91.4
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "biobuzz-blue-hive",
      "name": "Blue HIVE",
      "vertices": [
        {
          "x": 96.5,
          "y": 91.4
        },
        {
          "x": 93.9,
          "y": 91.4
        },
        {
          "x": 93.9,
          "y": 50
        },
        {
          "x": 96.5,
          "y": 50
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-top",
      "name": "Scoring-Side FLOWER",
      "vertices": [
        {
          "x": 43.4,
          "y": 141.5
        },
        {
          "x": 50.1,
          "y": 141.5
        },
        {
          "x": 50.1,
          "y": 139.4
        },
        {
          "x": 46.75,
          "y": 137.7
        },
        {
          "x": 43.4,
          "y": 139.4
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "biobuzz-flower-right",
      "name": "Blue Alliance FLOWER",
      "vertices": [
        {
          "x": 141.5,
          "y": 98.1
        },
        {
          "x": 139.4,
          "y": 98.1
        },
        {
          "x": 137.7,
          "y": 94.75
        },
        {
          "x": 139.4,
          "y": 91.4
        },
        {
          "x": 141.5,
          "y": 91.4
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-bottom",
      "name": "Audience-Side FLOWER",
      "vertices": [
        {
          "x": 91.15,
          "y": 0
        },
        {
          "x": 97.8,
          "y": 0
        },
        {
          "x": 97.8,
          "y": 2.1
        },
        {
          "x": 94.5,
          "y": 3.8
        },
        {
          "x": 91.15,
          "y": 2.1
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-left",
      "name": "Red Alliance FLOWER",
      "vertices": [
        {
          "x": 0,
          "y": 50.35
        },
        {
          "x": 2.1,
          "y": 50.35
        },
        {
          "x": 3.8,
          "y": 47
        },
        {
          "x": 2.1,
          "y": 43.65
        },
        {
          "x": 0,
          "y": 43.65
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mwj0ewa8-p3iolw"
    },
    {
      "kind": "path",
      "lineId": "line-mwj0gt0j-1osaj1"
    },
    {
      "kind": "path",
      "lineId": "line-mwnyqe6s-ow0mhd"
    },
    {
      "kind": "path",
      "lineId": "line-mwnyr3z4-obw36b"
    },
    {
      "kind": "path",
      "lineId": "line-mwj0hanf-0rd3w3"
    },
    {
      "kind": "path",
      "lineId": "line-mwj0i0ei-3rzo3w"
    },
    {
      "kind": "path",
      "lineId": "line-mwj0iive-4e0rvh"
    }
  ],
  "fieldPoints": [],
  "robot": {
    "forwardVelocity": 75,
    "strafeVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rLength": 16,
    "rWidth": 14.4,
    "safetyMargin": 1,
    "maxAcceleration": 50,
    "maxDeceleration": 50
  },
  "activePaths": []
}