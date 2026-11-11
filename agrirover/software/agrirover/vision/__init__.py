"""Plant-monitoring vision pipeline.

Maturity labels used throughout the project:
  DEMONSTRATION  classical colour/shape pipeline in this package; verified on synthetic images only
  EXPERIMENTAL   ONNX detector hook (vision/onnx_detector.py); no trained weights are shipped
  VALIDATED      nothing yet - requires labelled greenhouse data (see docs/07_computer_vision.md)
Outputs are screening flags with confidence scores, never diagnoses.
"""
