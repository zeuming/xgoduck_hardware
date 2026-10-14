"""Generated from the pinned official walk ONNX metadata; no hardware I/O.

This deliberately records the model's HOME, which differs from the upstream
Python hardcoded +/-23 degrees. Calibration and the installed policy use the
same model-specific reference. No stored calibration is rewritten on import.
"""
WALK_MODEL_SHA256 = '5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f'
WALK_HOME_RAD = (0.0, -0.087, -0.349, -0.005, 0.349, 0.349, 0.349, 0.0, 0.0, 0.0, 0.087, 0.349, 0.005, -0.349)
WALK_DEFAULT_DEG = (0.0, -4.984732817638162, -19.99622705006573, -0.2864788975654116, 19.99622705006573, 0.0, 4.984732817638162, 19.99622705006573, 0.2864788975654116, -19.99622705006573, 19.99622705006573, 19.99622705006573, 0.0, 0.0, 0.0)
