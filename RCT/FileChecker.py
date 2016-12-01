import os
import wx
class FileChecker:
	@staticmethod
	def SD_check():
		if(os.path.isdir('F:/')):
			print "got SD"
		else:
			print "no SD"
			dialog = wx.MessageDialog(self, message = "SD Card not found, try again?", caption = "SD Card", style = wx.YES_NO, pos = wx.DefaultPosition)
			response = dialog.ShowModal()
			if (response == wx.ID_YES):
				self.OpenSD()
			else:
				raise SystemExit