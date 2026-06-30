const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  file: {
    open: () => ipcRenderer.invoke('file:open'),
    read: (filePath) => ipcRenderer.invoke('file:read', filePath),
  },
});
