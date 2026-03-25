def duty_from_intensity(intensity: float) -> float:
    intensity = max(0.0, min(1.0, intensity))
    return intensity * 100.0
