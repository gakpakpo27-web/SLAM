#!/usr/bin/env python3
"""Réinjecte lidar_core.js, test_lidar_core.js et src/maps_ui.js dans lidar_viewer.html (fichier autonome)."""
import re, pathlib
h = pathlib.Path('lidar_viewer.html'); s = h.read_text(encoding='utf-8')
core = pathlib.Path('lidar_core.js').read_text(encoding='utf-8')
tests = pathlib.Path('test_lidar_core.js').read_text(encoding='utf-8')
ui = pathlib.Path('src/maps_ui.js').read_text(encoding='utf-8')
a, b = s.index('<!-- CORE:BEGIN -->'), s.index('<!-- CORE:END -->')
s = s[:a] + '<!-- CORE:BEGIN -->\n<script>\n' + core + '\n</script>\n<script>\n' + tests + '\n</script>\n' + s[b:]
a, b = s.index('// MAPS-UI:BEGIN'), s.index('// MAPS-UI:END')
s = s[:a] + '// MAPS-UI:BEGIN\n' + ui + '\n' + s[b:]
h.write_text(s, encoding='utf-8'); print('lidar_viewer.html reconstruit')
