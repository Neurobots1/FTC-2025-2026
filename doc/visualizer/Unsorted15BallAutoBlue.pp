{
  "startPoint": {
    "x": 20,
    "y": 119,
    "heading": "constant",
    "degrees": 139,
    "locked": false
  },
  "lines": [
    {
      "id": "line-preload-shot",
      "name": "Start to Preload Shot",
      "endPoint": {
        "x": 47,
        "y": 92,
        "heading": "linear",
        "startDeg": 139,
        "endDeg": 139
      },
      "controlPoints": [],
      "color": "#22c55e",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-preload-to-line2-start",
      "name": "Preload Shot to Line 2 Start",
      "endPoint": {
        "x": 50,
        "y": 55,
        "heading": "linear",
        "startDeg": 139,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-line2-intake",
      "name": "Line 2 Intake Sweep",
      "endPoint": {
        "x": 13,
        "y": 55,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-line2-to-shot",
      "name": "Line 2 to Shot",
      "endPoint": {
        "x": 47,
        "y": 92,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 139
      },
      "controlPoints": [
        {
          "x": 45,
          "y": 55
        }
      ],
      "color": "#22c55e",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-shot-to-gate-1",
      "name": "Shot to Gate",
      "endPoint": {
        "x": 11,
        "y": 53,
        "heading": "linear",
        "startDeg": 139,
        "endDeg": 147
      },
      "controlPoints": [
        {
          "x": 27,
          "y": 50
        }
      ],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-gate-to-shot",
      "name": "Gate to Shot",
      "endPoint": {
        "x": 47,
        "y": 92,
        "heading": "linear",
        "startDeg": 147,
        "endDeg": 139
      },
      "controlPoints": [
        {
          "x": 45,
          "y": 55
        }
      ],
      "color": "#22c55e",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-shot-to-gate-2",
      "name": "Shot to Gate Again",
      "endPoint": {
        "x": 11,
        "y": 53,
        "heading": "linear",
        "startDeg": 139,
        "endDeg": 147
      },
      "controlPoints": [
        {
          "x": 27,
          "y": 50
        }
      ],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-gate-to-final-shot",
      "name": "Gate to Final Shot",
      "endPoint": {
        "x": 55,
        "y": 105,
        "heading": "linear",
        "startDeg": 147,
        "endDeg": 145
      },
      "controlPoints": [
        {
          "x": 45,
          "y": 55
        }
      ],
      "color": "#22c55e",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-final-shot-to-line1-start",
      "name": "Final Shot to Line 1 Start",
      "endPoint": {
        "x": 50,
        "y": 88,
        "heading": "linear",
        "startDeg": 145,
        "endDeg": 190
      },
      "controlPoints": [],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-line1-intake",
      "name": "Line 1 Intake Sweep",
      "endPoint": {
        "x": 23,
        "y": 82,
        "heading": "linear",
        "startDeg": 190,
        "endDeg": 190
      },
      "controlPoints": [],
      "color": "#f97316",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-line1-to-final-shot",
      "name": "Line 1 to Final Shot",
      "endPoint": {
        "x": 55,
        "y": 105,
        "heading": "linear",
        "startDeg": 190,
        "endDeg": 145
      },
      "controlPoints": [],
      "color": "#22c55e",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 120,
          "y": 141.5
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-preload-shot"
    },
    {
      "kind": "path",
      "lineId": "line-preload-to-line2-start"
    },
    {
      "kind": "path",
      "lineId": "line-line2-intake"
    },
    {
      "kind": "wait",
      "id": "wait-line2-intake-settle",
      "name": "Line 2 Intake Settle",
      "durationMs": 900,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "line-line2-to-shot"
    },
    {
      "kind": "path",
      "lineId": "line-shot-to-gate-1"
    },
    {
      "kind": "wait",
      "id": "wait-gate-collect-1",
      "name": "Gate Collect",
      "durationMs": 1500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "line-gate-to-shot"
    },
    {
      "kind": "path",
      "lineId": "line-shot-to-gate-2"
    },
    {
      "kind": "wait",
      "id": "wait-gate-collect-2",
      "name": "Gate Collect Again",
      "durationMs": 1500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "line-gate-to-final-shot"
    },
    {
      "kind": "path",
      "lineId": "line-final-shot-to-line1-start"
    },
    {
      "kind": "path",
      "lineId": "line-line1-intake"
    },
    {
      "kind": "wait",
      "id": "wait-line1-intake-settle",
      "name": "Line 1 Intake Settle",
      "durationMs": 900,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "line-line1-to-final-shot"
    }
  ],
  "pathChains": [
    {
      "id": "chain-unsorted-15-ball-blue",
      "name": "Unsorted 15 Ball Auto Blue",
      "color": "#2563eb",
      "lineIds": [
        "line-preload-shot",
        "line-preload-to-line2-start",
        "line-line2-intake",
        "line-line2-to-shot",
        "line-shot-to-gate-1",
        "line-gate-to-shot",
        "line-shot-to-gate-2",
        "line-gate-to-final-shot",
        "line-final-shot-to-line1-start",
        "line-line1-intake",
        "line-line1-to-final-shot"
      ]
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-04-11T00:00:00.000Z"
}
