# GUI_pages/__init__.py
from .HomePage import HomePage
from .ProgramEditorPage import ProgramEditorPage
from .SimulationPage import SimulationPage
from .ConsolePage import ConsolePage
from .ConnectionPage import ConnectionPage
from .ProjectManagerPage import ProjectManagerPage
from .DiagnosticsPage import DiagnosticsPage
from .SettingsPage import SettingsPage
# from .OtherPage import OtherPage

ALL_PROGRAM_PAGES = [HomePage, ProgramEditorPage, SimulationPage,
                     ConsolePage, ConnectionPage, ProjectManagerPage,
                     DiagnosticsPage, SettingsPage]