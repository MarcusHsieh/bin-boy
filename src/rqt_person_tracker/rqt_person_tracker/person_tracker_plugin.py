#!/usr/bin/env python3
"""
RQT Person Tracker Plugin
Main plugin class for RQT integration
"""

from rqt_gui_py.plugin import Plugin
from .person_tracker_widget import PersonTrackerWidget


class PersonTrackerPlugin(Plugin):
    """RQT plugin for person tracking dashboard"""

    def __init__(self, context):
        super(PersonTrackerPlugin, self).__init__(context)
        self.setObjectName('PersonTrackerPlugin')

        # Create widget
        self._widget = PersonTrackerWidget(context.node)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number())
            )

        # Add widget to GUI
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        """Cleanup on shutdown"""
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings"""
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings"""
        pass
