"""Design tokens for the DUBO GCS ("Dark Ocean" glassmorphism).

Pulled from stitch_hydrion_spectra_gcs/hydrion_spectra_gcs/DESIGN.md.
"""

C = {
    "bg": "#041329",
    "bg_lowest": "#010e24",
    "surface_low": "#0d1c32",
    "surface": "#112036",
    "surface_high": "#1c2a41",
    "surface_highest": "#27354c",
    "on_surface": "#d6e3ff",
    "on_surface_variant": "#bacac3",
    "outline": "#85948e",
    "outline_variant": "#3c4a45",
    "primary": "#64ffda",
    "primary_dim": "#38debb",
    "on_primary": "#00382d",
    "secondary": "#adc7ff",
    "on_secondary": "#002e68",
    "error": "#ffb4ab",
    "on_error": "#690005",
    "error_container": "#93000a",
    "on_error_container": "#ffdad6",
    "divider": "#233554",
    "glass_bg": "rgba(17, 34, 64, 0.72)",
    "glass_border": "rgba(100, 255, 218, 0.22)",
    "glass_glow": "rgba(100, 255, 218, 0.10)",
}

FONT_UI = "'Inter', 'Noto Sans', 'DejaVu Sans', sans-serif"
FONT_DATA = "'JetBrains Mono', 'DejaVu Sans Mono', monospace"


def ui(color):
    return C[color]


def apply_theme(app):
    """Apply the global DUBO stylesheet to the application."""
    app.setStyleSheet(_GLOBAL_QSS)


_GLOBAL_QSS = f"""
    QMainWindow, QWidget {{ background: {C['bg']}; color: {C['on_surface']};
        font-family: {FONT_UI}; font-size: 13px; }}
    QLabel {{ background: transparent; }}
    QFrame#GlassPanel {{
        background-color: {C['glass_bg']};
        border: 1px solid {C['glass_border']};
        border-radius: 12px;
    }}
    QLabel#GlassTitle {{
        background: {C['surface_high']};
        border-top-left-radius: 12px; border-top-right-radius: 12px;
        color: {C['on_surface']}; font-weight: 700; font-size: 11px;
        letter-spacing: 2px; padding: 4px 8px;
    }}
    QLabel#CapsLabel {{ color: {C['on_surface_variant']}; font-weight: 700;
        font-size: 11px; letter-spacing: 2px; }}
    QLabel#DataValue {{ font-family: {FONT_DATA}; font-size: 14px; color: {C['primary']}; }}
    QLabel#DataValueLg {{ font-family: {FONT_DATA}; font-size: 20px; color: {C['primary']}; }}
    QLabel#DataDim {{ font-family: {FONT_DATA}; font-size: 12px; color: {C['on_surface_variant']}; }}
    QLabel#Brand {{ font-family: {FONT_UI}; font-weight: 700; font-size: 20px;
        color: {C['primary']}; letter-spacing: 1px; }}
    QLabel#StatusPill {{
        border-radius: 10px; padding: 2px 10px; font-weight: 700; font-size: 11px;
        letter-spacing: 1px; }}
    QPushButton {{
        background: {C['surface']}; color: {C['on_surface']};
        border: 1px solid {C['outline_variant']}; border-radius: 8px;
        padding: 5px 12px; font-weight: 600; }}
    QPushButton:hover {{ background: {C['surface_high']};
        border-color: {C['primary']}; color: {C['primary']}; }}
    QPushButton:checked {{
        background: {C['primary']}; color: {C['on_primary']}; border: 1px solid {C['primary']}; }}
    QPushButton#Primary {{
        background: {C['primary']}; color: {C['on_primary']}; border: 1px solid {C['primary']}; }}
    QPushButton#Primary:hover {{ background: {C['primary_dim']}; }}
    QPushButton#Danger {{
        background: {C['error_container']}; color: {C['on_error_container']};
        border: 1px solid rgba(255, 180, 171, 0.5); }}
    QPushButton#Danger:hover {{ background: {C['error']}; color: {C['on_error']}; }}
    QPushButton#Ghost {{ background: transparent; color: {C['on_surface_variant']};
        border: 1px solid {C['outline_variant']}; }}
    QPushButton#NavBtn {{
        background: transparent; border: none; border-radius: 8px; text-align: left;
        color: {C['on_surface_variant']}; }}
    QPushButton#NavBtn:hover {{ background: rgba(100, 255, 218, 0.10);
        color: {C['primary']}; }}
    QPushButton#NavBtn:checked {{ background: {C['primary']}; color: {C['on_primary']}; }}
    QLineEdit, QSpinBox, QComboBox {{
        background: {C['bg_lowest']}; color: {C['on_surface']};
        border: 1px solid {C['outline_variant']}; border-radius: 8px; padding: 4px 8px; }}
    QLineEdit:focus {{ border-color: {C['primary']}; }}
    QTextEdit {{ background: {C['bg_lowest']}; color: {C['on_surface']};
        border: 1px solid {C['outline_variant']}; border-radius: 8px;
        font-family: {FONT_DATA}; font-size: 12px; }}
    QScrollBar:vertical {{ background: transparent; width: 8px; }}
    QScrollBar::handle:vertical {{ background: {C['surface_highest']}; border-radius: 4px;
        min-height: 24px; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
    QStatusBar {{ background: {C['bg_lowest']}; border-top: 1px solid
        rgba(100, 255, 218, 0.15); color: {C['on_surface_variant']}; }}
    """
