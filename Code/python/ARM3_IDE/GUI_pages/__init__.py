# GUI_pages/__init__.py
from GUI_pages.GUI_template import PageBase
from GUI_pages.HomePage import HomePage
from GUI_pages.ProgramEditorPage import ProgramEditorPage
from GUI_pages.SimulationPage import SimulationPage
from GUI_pages.ConsolePage import ConsolePage
from GUI_pages.ConnectionPage import ConnectionPage
from GUI_pages.ProjectManagerPage import ProjectManagerPage
from GUI_pages.DiagnosticsPage import DiagnosticsPage
from GUI_pages.SettingsPage import SettingsPage
# from .OtherPage import OtherPage

ALL_PROGRAM_PAGES: list[PageBase] = [HomePage, ProgramEditorPage, SimulationPage,
                     ConsolePage, ConnectionPage, ProjectManagerPage,
                     DiagnosticsPage, SettingsPage]