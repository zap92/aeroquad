<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="9008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Dialog" Type="Folder" URL="../Dialog">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Utilities" Type="Folder" URL="../Utilities">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Icon" Type="Folder" URL="../Icon">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="AeroQuadConfigurator.vi" Type="VI" URL="../AeroQuadConfigurator.vi"/>
		<Item Name="AeroQuadConfigurator.ini" Type="Document" URL="../AeroQuadConfigurator.ini"/>
		<Item Name="AeroQuadLauncher.vi" Type="VI" URL="../AeroQuadLauncher.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Error Code Database.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Code Database.vi"/>
				<Item Name="Type Enum.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/GetType.llb/Type Enum.ctl"/>
				<Item Name="VariantType.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/VariantDataType/VariantType.lvlib"/>
				<Item Name="Waveform Duration.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/Waveform Duration.vi"/>
				<Item Name="WDT Waveform Duration DBL.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Waveform Duration DBL.vi"/>
				<Item Name="Number of Waveform Samples.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/Number of Waveform Samples.vi"/>
				<Item Name="WDT Number of Waveform Samples DBL.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples DBL.vi"/>
				<Item Name="WDT Number of Waveform Samples CDB.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples CDB.vi"/>
				<Item Name="WDT Number of Waveform Samples EXT.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples EXT.vi"/>
				<Item Name="WDT Number of Waveform Samples I16.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples I16.vi"/>
				<Item Name="WDT Number of Waveform Samples I32.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples I32.vi"/>
				<Item Name="WDT Number of Waveform Samples I8.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples I8.vi"/>
				<Item Name="WDT Number of Waveform Samples SGL.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/WDTOps.llb/WDT Number of Waveform Samples SGL.vi"/>
				<Item Name="DWDT Waveform Duration.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DWDTOps.llb/DWDT Waveform Duration.vi"/>
				<Item Name="DWDT Digital Size.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DWDTOps.llb/DWDT Digital Size.vi"/>
				<Item Name="DTbl Digital Size.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DTblOps.llb/DTbl Digital Size.vi"/>
				<Item Name="Uncompress Digital.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DWDT.llb/Uncompress Digital.vi"/>
				<Item Name="DWDT Uncompress Digital.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DWDTOps.llb/DWDT Uncompress Digital.vi"/>
				<Item Name="DTbl Uncompress Digital.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DTblOps.llb/DTbl Uncompress Digital.vi"/>
				<Item Name="Digital Size.vi" Type="VI" URL="/&lt;vilib&gt;/Waveform/DWDT.llb/Digital Size.vi"/>
				<Item Name="General Error Handler.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/General Error Handler.vi"/>
				<Item Name="DialogType.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/DialogType.ctl"/>
				<Item Name="DialogTypeEnum.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/DialogTypeEnum.ctl"/>
				<Item Name="General Error Handler CORE.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/General Error Handler CORE.vi"/>
				<Item Name="Check Special Tags.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Check Special Tags.vi"/>
				<Item Name="TagReturnType.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/TagReturnType.ctl"/>
				<Item Name="Set String Value.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set String Value.vi"/>
				<Item Name="GetRTHostConnectedProp.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/GetRTHostConnectedProp.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="Format Message String.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Format Message String.vi"/>
				<Item Name="Find Tag.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Find Tag.vi"/>
				<Item Name="Search and Replace Pattern.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Search and Replace Pattern.vi"/>
				<Item Name="Set Bold Text.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set Bold Text.vi"/>
				<Item Name="Details Display Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Details Display Dialog.vi"/>
				<Item Name="ErrWarn.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/ErrWarn.ctl"/>
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="eventvkey.ctl" Type="VI" URL="/&lt;vilib&gt;/event_ctls.llb/eventvkey.ctl"/>
				<Item Name="Not Found Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Not Found Dialog.vi"/>
				<Item Name="Three Button Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Three Button Dialog.vi"/>
				<Item Name="Three Button Dialog CORE.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Three Button Dialog CORE.vi"/>
				<Item Name="Longest Line Length in Pixels.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Longest Line Length in Pixels.vi"/>
				<Item Name="Convert property node font to graphics font.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Convert property node font to graphics font.vi"/>
				<Item Name="Get Text Rect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Get Text Rect.vi"/>
				<Item Name="Get String Text Bounds.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Get String Text Bounds.vi"/>
				<Item Name="LVBoundsTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVBoundsTypeDef.ctl"/>
				<Item Name="BuildHelpPath.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/BuildHelpPath.vi"/>
				<Item Name="GetHelpDir.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/GetHelpDir.vi"/>
				<Item Name="subTimeDelay.vi" Type="VI" URL="/&lt;vilib&gt;/express/express execution control/TimeDelayBlock.llb/subTimeDelay.vi"/>
				<Item Name="NI_LVConfig.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/config.llb/NI_LVConfig.lvlib"/>
				<Item Name="Check if File or Folder Exists.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/libraryn.llb/Check if File or Folder Exists.vi"/>
				<Item Name="NI_FileType.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/lvfile.llb/NI_FileType.lvlib"/>
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="Space Constant.vi" Type="VI" URL="/&lt;vilib&gt;/dlg_ctls.llb/Space Constant.vi"/>
				<Item Name="VISA Configure Serial Port" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port"/>
				<Item Name="VISA Configure Serial Port (Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Instr).vi"/>
				<Item Name="VISA Configure Serial Port (Serial Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Serial Instr).vi"/>
				<Item Name="NI_PtbyPt.lvlib" Type="Library" URL="/&lt;vilib&gt;/ptbypt/NI_PtbyPt.lvlib"/>
				<Item Name="NI_AALPro.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALPro.lvlib"/>
				<Item Name="Simple Error Handler.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Simple Error Handler.vi"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="Registry RtKey.ctl" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry RtKey.ctl"/>
				<Item Name="Close Registry Key.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Close Registry Key.vi"/>
				<Item Name="Registry refnum.ctl" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry refnum.ctl"/>
				<Item Name="Registry Handle Master.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry Handle Master.vi"/>
				<Item Name="Read Registry Value Simple.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value Simple.vi"/>
				<Item Name="Read Registry Value Simple STR.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value Simple STR.vi"/>
				<Item Name="Registry Simplify Data Type.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry Simplify Data Type.vi"/>
				<Item Name="Read Registry Value.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value.vi"/>
				<Item Name="Read Registry Value STR.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value STR.vi"/>
				<Item Name="Registry WinErr-LVErr.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry WinErr-LVErr.vi"/>
				<Item Name="Read Registry Value DWORD.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value DWORD.vi"/>
				<Item Name="Read Registry Value Simple U32.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Read Registry Value Simple U32.vi"/>
				<Item Name="Open Registry Key.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Open Registry Key.vi"/>
				<Item Name="Registry SAM.ctl" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry SAM.ctl"/>
				<Item Name="Registry View.ctl" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/Registry View.ctl"/>
				<Item Name="STR_ASCII-Unicode.vi" Type="VI" URL="/&lt;vilib&gt;/registry/registry.llb/STR_ASCII-Unicode.vi"/>
				<Item Name="fileViewerConfigData.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/fileViewerConfigData.ctl"/>
				<Item Name="initFileContentsTree.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/initFileContentsTree.vi"/>
				<Item Name="status.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/status.vi"/>
				<Item Name="TDMSFileViewerLocalizedText.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/TDMSFileViewerLocalizedText.vi"/>
				<Item Name="setListBoxColumnWidths.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/setListBoxColumnWidths.vi"/>
				<Item Name="loadAndFormatValues.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/loadAndFormatValues.vi"/>
				<Item Name="ClearError.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/ClearError.vi"/>
				<Item Name="getChannelList.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/getChannelList.vi"/>
				<Item Name="formatPropertyList.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/formatPropertyList.vi"/>
				<Item Name="Get Type of Variant.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/GetType.llb/Get Type of Variant.vi"/>
				<Item Name="VariantType to Type Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/GetType.llb/VariantType to Type Code.vi"/>
				<Item Name="getNamesFromPath.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/getNamesFromPath.vi"/>
				<Item Name="configureNumberOfValues.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/configureNumberOfValues.vi"/>
				<Item Name="panelResize.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/tdmsutil.llb/panelResize.vi"/>
				<Item Name="LVRectTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVRectTypeDef.ctl"/>
				<Item Name="subFile Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/express/express input/FileDialogBlock.llb/subFile Dialog.vi"/>
				<Item Name="ex_CorrectErrorChain.vi" Type="VI" URL="/&lt;vilib&gt;/express/express shared/ex_CorrectErrorChain.vi"/>
				<Item Name="3D Scatter Datatype.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Scatter/3D Scatter Datatype/3D Scatter Datatype.lvclass"/>
				<Item Name="3D Plot Datatype.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/3D Plot Datatype/3D Plot Datatype.lvclass"/>
				<Item Name="3D Plot.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/3D Plot/3D Plot.lvclass"/>
				<Item Name="3DMathPlot Ctrl Act Cluster.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Action String/3DMathPlot Ctrl Act Cluster.ctl"/>
				<Item Name="3DMathPlot State Class.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Action String/3DMathPlot State Class.ctl"/>
				<Item Name="NI_3D Picture Control.lvlib" Type="Library" URL="/&lt;vilib&gt;/picture/3D Picture Control/NI_3D Picture Control.lvlib"/>
				<Item Name="3DMathPlot Ctrl Act Queue.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Action String/3DMathPlot Ctrl Act Queue.ctl"/>
				<Item Name="Color to RGB.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/Color to RGB.vi"/>
				<Item Name="NI_Math Plot Private Lib.lvlib" Type="Library" URL="/&lt;vilib&gt;/Math Plots/Plot Private Lib/NI_Math Plot Private Lib.lvlib"/>
				<Item Name="RGB to Color.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/RGB to Color.vi"/>
				<Item Name="Draw Flattened Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Flattened Pixmap.vi"/>
				<Item Name="imagedata.ctl" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/imagedata.ctl"/>
				<Item Name="FixBadRect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/FixBadRect.vi"/>
				<Item Name="LV3DPointTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LV3DPointTypeDef.ctl"/>
				<Item Name="New.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/New.ctl"/>
				<Item Name="Delete.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Delete.ctl"/>
				<Item Name="Move Up.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Move Up.ctl"/>
				<Item Name="Move Down.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Move Down.ctl"/>
				<Item Name="Point Style Ring.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Point Style Ring.ctl"/>
				<Item Name="Line Style Ring.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Line Style Ring.ctl"/>
				<Item Name="Color Slider.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/Color Slider.ctl"/>
				<Item Name="3D Scatter.xctl" Type="XControl" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Scatter/3D Scatter XCtrl/3D Scatter.xctl"/>
				<Item Name="XControlSupport.lvlib" Type="Library" URL="/&lt;vilib&gt;/_xctls/XControlSupport.lvlib"/>
				<Item Name="3D Scatter.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Scatter/3D Scatter/3D Scatter.lvclass"/>
				<Item Name="Merge Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Merge Errors.vi"/>
				<Item Name="XY Projection.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/XY Projection.ctl"/>
				<Item Name="XZ Projection.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/XZ Projection.ctl"/>
				<Item Name="YZ Projection.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/YZ Projection.ctl"/>
				<Item Name="3D Projection.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Buttons/3D Projection.ctl"/>
				<Item Name="3DMathPlot Action String.ctl" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Action String/3DMathPlot Action String.ctl"/>
				<Item Name="3DMathPlot Action String.vi" Type="VI" URL="/&lt;vilib&gt;/Math Plots/3D Math Plots/3D Plot/Action String/3DMathPlot Action String.vi"/>
				<Item Name="Version To Dotted String.vi" Type="VI" URL="/&lt;vilib&gt;/_xctls/Version To Dotted String.vi"/>
			</Item>
			<Item Name="Advapi32.dll" Type="Document" URL="Advapi32.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="kernel32.dll" Type="Document" URL="kernel32.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="../../../../../../../../Program Files/National Instruments/LabVIEW 2009/resource/lvanlys.dll"/>
			<Item Name="_LaunchHelp.vi" Type="VI" URL="/&lt;helpdir&gt;/_LaunchHelp.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="Executable" Type="EXE">
				<Property Name="App_applicationGUID" Type="Str">{66C02D92-29AD-43F8-A6C3-B835B0C6FAEA}</Property>
				<Property Name="App_applicationName" Type="Str">AeroQuadConfigurator.exe</Property>
				<Property Name="App_companyName" Type="Str">Carancho Engineering LLC</Property>
				<Property Name="App_fileDescription" Type="Str">Executable
2.6.0.0
Copyright 2010 AeroQuad</Property>
				<Property Name="App_fileVersion.major" Type="Int">2</Property>
				<Property Name="App_fileVersion.minor" Type="Int">6</Property>
				<Property Name="App_INI_aliasGUID" Type="Str">{0E512C75-3384-479B-B8B3-5B0BF1B031D9}</Property>
				<Property Name="App_INI_GUID" Type="Str">{95AB525D-9CC0-48EA-84D7-D36AB922FE00}</Property>
				<Property Name="App_INI_itemID" Type="Ref">/My Computer/AeroQuadConfigurator.ini</Property>
				<Property Name="App_internalName" Type="Str">Executable</Property>
				<Property Name="App_legalCopyright" Type="Str">Copyright 2010 AeroQuad</Property>
				<Property Name="App_productName" Type="Str">AeroQuad Configurator</Property>
				<Property Name="Bld_buildSpecName" Type="Str">Executable</Property>
				<Property Name="Bld_excludeLibraryItems" Type="Bool">true</Property>
				<Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
				<Property Name="Bld_excludeTypedefs" Type="Bool">true</Property>
				<Property Name="Bld_modifyLibraryFile" Type="Bool">true</Property>
				<Property Name="Bld_supportedLanguage[0]" Type="Str">English</Property>
				<Property Name="Bld_supportedLanguageCount" Type="Int">1</Property>
				<Property Name="Destination[0].destName" Type="Str">AeroQuadConfigurator.app</Property>
				<Property Name="Destination[0].path" Type="Path">../Executable/NI_AB_PROJECTNAME.exe</Property>
				<Property Name="Destination[0].type" Type="Str">App</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">../Executable/data</Property>
				<Property Name="Destination[2].destName" Type="Str">DLL Directory</Property>
				<Property Name="Destination[2].path" Type="Path">../Executable</Property>
				<Property Name="DestinationCount" Type="Int">3</Property>
				<Property Name="Exe_iconItemID" Type="Ref">/My Computer/Icon/transparent icon.ico</Property>
				<Property Name="Source[0].itemID" Type="Str">{AE3B1FEE-C0D9-4713-958B-2F2FB7F4E866}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/AeroQuadConfigurator.ini</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[2].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[2].itemID" Type="Ref">/My Computer/AeroQuadLauncher.vi</Property>
				<Property Name="Source[2].sourceInclusion" Type="Str">TopLevel</Property>
				<Property Name="Source[2].type" Type="Str">VI</Property>
				<Property Name="Source[3].destinationIndex" Type="Int">1</Property>
				<Property Name="Source[3].itemID" Type="Ref">/My Computer/AeroQuadConfigurator.vi</Property>
				<Property Name="Source[3].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[3].type" Type="Str">VI</Property>
				<Property Name="SourceCount" Type="Int">4</Property>
			</Item>
			<Item Name="Installer" Type="Installer">
				<Property Name="arpCompany" Type="Str">Carancho Engineering LLC</Property>
				<Property Name="arpContact" Type="Str">Ted Carancho</Property>
				<Property Name="arpURL" Type="Str">http://www.AeroQuad.com</Property>
				<Property Name="AutoIncrement" Type="Bool">true</Property>
				<Property Name="BldInfo.Count" Type="Int">1</Property>
				<Property Name="BldInfo[0].Dir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="BldInfo[0].Tag" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="BuildLabel" Type="Str">Installer</Property>
				<Property Name="BuildLocation" Type="Path">../../builds/AeroQuadConfigurator/Installer</Property>
				<Property Name="DirInfo.Count" Type="Int">2</Property>
				<Property Name="DirInfo.DefaultDir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DirInfo[0].DirName" Type="Str">AeroQuadConfigurator</Property>
				<Property Name="DirInfo[0].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DirInfo[0].ParentTag" Type="Str">{3912416A-D2E5-411B-AFEE-B63654D690C0}</Property>
				<Property Name="DirInfo[1].DirName" Type="Str">data</Property>
				<Property Name="DirInfo[1].DirTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[1].ParentTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DistID" Type="Str">{AC943C01-8EA8-4BFF-A46F-CC5E452DF24E}</Property>
				<Property Name="DistParts.Count" Type="Int">2</Property>
				<Property Name="DistPartsInfo[0].FlavorID" Type="Str">DefaultFull</Property>
				<Property Name="DistPartsInfo[0].ProductID" Type="Str">{7312C1D1-70C8-400E-9CAF-AF1DD2763269}</Property>
				<Property Name="DistPartsInfo[0].ProductName" Type="Str">NI LabVIEW Run-Time Engine 2009 SP1</Property>
				<Property Name="DistPartsInfo[0].UpgradeCode" Type="Str">{1DA01FF3-1E36-4A14-802B-D195819E159B}</Property>
				<Property Name="DistPartsInfo[1].FlavorID" Type="Str">_deployment_</Property>
				<Property Name="DistPartsInfo[1].ProductID" Type="Str">{D6FC9FA9-3386-409A-8D62-EE026CA721D1}</Property>
				<Property Name="DistPartsInfo[1].ProductName" Type="Str">NI-VISA Runtime 4.5</Property>
				<Property Name="DistPartsInfo[1].UpgradeCode" Type="Str">{8627993A-3F66-483C-A562-0D3BA3F267B1}</Property>
				<Property Name="FileInfo.Count" Type="Int">4</Property>
				<Property Name="FileInfo[0].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[0].FileName" Type="Str">AeroQuadConfigurator.exe</Property>
				<Property Name="FileInfo[0].FileTag" Type="Str">{66C02D92-29AD-43F8-A6C3-B835B0C6FAEA}</Property>
				<Property Name="FileInfo[0].Type" Type="Int">3</Property>
				<Property Name="FileInfo[0].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[1].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[1].FileName" Type="Str">AeroQuadConfigurator.aliases</Property>
				<Property Name="FileInfo[1].FileTag" Type="Str">{0E512C75-3384-479B-B8B3-5B0BF1B031D9}</Property>
				<Property Name="FileInfo[1].Type" Type="Int">3</Property>
				<Property Name="FileInfo[1].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[2].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[2].FileName" Type="Str">AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[2].FileTag" Type="Str">{95AB525D-9CC0-48EA-84D7-D36AB922FE00}</Property>
				<Property Name="FileInfo[2].Type" Type="Int">3</Property>
				<Property Name="FileInfo[2].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[3].DirTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="FileInfo[3].FileName" Type="Str">AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[3].FileTag" Type="Ref">/My Computer/AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[3].Type" Type="Int">3</Property>
				<Property Name="FileInfo[3].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="InstSpecVersion" Type="Str">9018011</Property>
				<Property Name="LicenseFile" Type="Ref"></Property>
				<Property Name="OSCheck" Type="Int">0</Property>
				<Property Name="OSCheck_Vista" Type="Bool">false</Property>
				<Property Name="ProductName" Type="Str">AeroQuad Configurator</Property>
				<Property Name="ProductVersion" Type="Str">2.6.5</Property>
				<Property Name="ReadmeFile" Type="Ref"></Property>
				<Property Name="ShortcutInfo.Count" Type="Int">1</Property>
				<Property Name="ShortcutInfo[0].DirTag" Type="Str">{B9E310F1-839C-48B7-8CAE-33000780C26E}</Property>
				<Property Name="ShortcutInfo[0].FileTag" Type="Str">{66C02D92-29AD-43F8-A6C3-B835B0C6FAEA}</Property>
				<Property Name="ShortcutInfo[0].FileTagDir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="ShortcutInfo[0].Name" Type="Str">AeroQuad Configurator</Property>
				<Property Name="ShortcutInfo[0].SubDir" Type="Str">AeroQuad Configurator</Property>
				<Property Name="UpgradeCode" Type="Str">{24029CB6-D54A-4D61-8023-884E5516E281}</Property>
				<Property Name="WindowMessage" Type="Str">Thanks for using the AeroQuad Configurator.  The Configurator is used to setup your AeroQuad.  If you have any questions or suggestions for improvement for the Configurator, please post them at http://AeroQuad.com/forum.php</Property>
				<Property Name="WindowTitle" Type="Str">AeroQuad Configurator</Property>
			</Item>
			<Item Name="Updater" Type="Installer">
				<Property Name="arpCompany" Type="Str">Carancho Engineering LLC</Property>
				<Property Name="arpContact" Type="Str">Ted Carancho</Property>
				<Property Name="arpURL" Type="Str">http://www.AeroQuad.com</Property>
				<Property Name="AutoIncrement" Type="Bool">true</Property>
				<Property Name="BldInfo.Count" Type="Int">1</Property>
				<Property Name="BldInfo[0].Dir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="BldInfo[0].Tag" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="BuildLabel" Type="Str">Updater</Property>
				<Property Name="BuildLocation" Type="Path">../../builds/AeroQuadConfigurator/Updater</Property>
				<Property Name="DirInfo.Count" Type="Int">7</Property>
				<Property Name="DirInfo.DefaultDir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DirInfo[0].DirName" Type="Str">AeroQuadConfigurator</Property>
				<Property Name="DirInfo[0].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DirInfo[0].ParentTag" Type="Str">{3912416A-D2E5-411B-AFEE-B63654D690C0}</Property>
				<Property Name="DirInfo[1].DirName" Type="Str">data</Property>
				<Property Name="DirInfo[1].DirTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[1].ParentTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="DirInfo[2].DirName" Type="Str">3D Plot</Property>
				<Property Name="DirInfo[2].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="DirInfo[2].ParentTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[3].DirName" Type="Str">3D Plot Datatype</Property>
				<Property Name="DirInfo[3].DirTag" Type="Str">{0051B67B-C11C-4737-8852-03A772D3AF14}</Property>
				<Property Name="DirInfo[3].ParentTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[4].DirName" Type="Str">3D Scatter</Property>
				<Property Name="DirInfo[4].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="DirInfo[4].ParentTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[5].DirName" Type="Str">3D Scatter Datatype</Property>
				<Property Name="DirInfo[5].DirTag" Type="Str">{003DAD04-8A37-45BD-840C-25A67C790C1B}</Property>
				<Property Name="DirInfo[5].ParentTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DirInfo[6].DirName" Type="Str">NI_3D Picture Control</Property>
				<Property Name="DirInfo[6].DirTag" Type="Str">{4A68D4FE-0F2D-4EEE-9E57-C6E5EEE95FFF}</Property>
				<Property Name="DirInfo[6].ParentTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="DistID" Type="Str">{C4477131-6271-4257-B5A0-868DA892ED61}</Property>
				<Property Name="FileInfo.Count" Type="Int">327</Property>
				<Property Name="FileInfo[0].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[0].FileName" Type="Str">AeroQuadConfigurator.exe</Property>
				<Property Name="FileInfo[0].FileTag" Type="Str">{66C02D92-29AD-43F8-A6C3-B835B0C6FAEA}</Property>
				<Property Name="FileInfo[0].Type" Type="Int">3</Property>
				<Property Name="FileInfo[0].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[1].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[1].FileName" Type="Str">AeroQuadConfigurator.aliases</Property>
				<Property Name="FileInfo[1].FileTag" Type="Str">{0E512C75-3384-479B-B8B3-5B0BF1B031D9}</Property>
				<Property Name="FileInfo[1].Type" Type="Int">3</Property>
				<Property Name="FileInfo[1].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[10].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[10].FileName" Type="Str">Projection Mode Write.vi</Property>
				<Property Name="FileInfo[10].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Projection Mode Write.vi</Property>
				<Property Name="FileInfo[10].Type" Type="Int">3</Property>
				<Property Name="FileInfo[10].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[100].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[100].FileName" Type="Str">Active Cursor Write.vi</Property>
				<Property Name="FileInfo[100].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Active Cursor Write.vi</Property>
				<Property Name="FileInfo[100].Type" Type="Int">3</Property>
				<Property Name="FileInfo[100].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[101].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[101].FileName" Type="Str">Cursor Enable Read.vi</Property>
				<Property Name="FileInfo[101].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Enable Read.vi</Property>
				<Property Name="FileInfo[101].Type" Type="Int">3</Property>
				<Property Name="FileInfo[101].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[102].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[102].FileName" Type="Str">Cursor Enable Write.vi</Property>
				<Property Name="FileInfo[102].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Enable Write.vi</Property>
				<Property Name="FileInfo[102].Type" Type="Int">3</Property>
				<Property Name="FileInfo[102].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[103].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[103].FileName" Type="Str">Cursor Visible Read.vi</Property>
				<Property Name="FileInfo[103].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Visible Read.vi</Property>
				<Property Name="FileInfo[103].Type" Type="Int">3</Property>
				<Property Name="FileInfo[103].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[104].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[104].FileName" Type="Str">Cursor Visible Write.vi</Property>
				<Property Name="FileInfo[104].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Visible Write.vi</Property>
				<Property Name="FileInfo[104].Type" Type="Int">3</Property>
				<Property Name="FileInfo[104].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[105].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[105].FileName" Type="Str">Cursor Plot ID Read.vi</Property>
				<Property Name="FileInfo[105].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plot ID Read.vi</Property>
				<Property Name="FileInfo[105].Type" Type="Int">3</Property>
				<Property Name="FileInfo[105].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[106].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[106].FileName" Type="Str">Cursor Plot ID Write.vi</Property>
				<Property Name="FileInfo[106].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plot ID Write.vi</Property>
				<Property Name="FileInfo[106].Type" Type="Int">3</Property>
				<Property Name="FileInfo[106].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[107].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[107].FileName" Type="Str">Cursor X Position Read.vi</Property>
				<Property Name="FileInfo[107].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor X Position Read.vi</Property>
				<Property Name="FileInfo[107].Type" Type="Int">3</Property>
				<Property Name="FileInfo[107].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[108].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[108].FileName" Type="Str">Cursor X Position Write.vi</Property>
				<Property Name="FileInfo[108].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor X Position Write.vi</Property>
				<Property Name="FileInfo[108].Type" Type="Int">3</Property>
				<Property Name="FileInfo[108].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[109].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[109].FileName" Type="Str">Cursor Y Position Read.vi</Property>
				<Property Name="FileInfo[109].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Y Position Read.vi</Property>
				<Property Name="FileInfo[109].Type" Type="Int">3</Property>
				<Property Name="FileInfo[109].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[11].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[11].FileName" Type="Str">View Direction Read.vi</Property>
				<Property Name="FileInfo[11].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Direction Read.vi</Property>
				<Property Name="FileInfo[11].Type" Type="Int">3</Property>
				<Property Name="FileInfo[11].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[110].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[110].FileName" Type="Str">Cursor Y Position Write.vi</Property>
				<Property Name="FileInfo[110].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Y Position Write.vi</Property>
				<Property Name="FileInfo[110].Type" Type="Int">3</Property>
				<Property Name="FileInfo[110].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[111].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[111].FileName" Type="Str">Cursor Z Position Read.vi</Property>
				<Property Name="FileInfo[111].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Z Position Read.vi</Property>
				<Property Name="FileInfo[111].Type" Type="Int">3</Property>
				<Property Name="FileInfo[111].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[112].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[112].FileName" Type="Str">Cursor Z Position Write.vi</Property>
				<Property Name="FileInfo[112].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Z Position Write.vi</Property>
				<Property Name="FileInfo[112].Type" Type="Int">3</Property>
				<Property Name="FileInfo[112].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[113].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[113].FileName" Type="Str">Cursor Lock Style Read.vi</Property>
				<Property Name="FileInfo[113].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Lock Style Read.vi</Property>
				<Property Name="FileInfo[113].Type" Type="Int">3</Property>
				<Property Name="FileInfo[113].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[114].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[114].FileName" Type="Str">Cursor Lock Style Write.vi</Property>
				<Property Name="FileInfo[114].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Lock Style Write.vi</Property>
				<Property Name="FileInfo[114].Type" Type="Int">3</Property>
				<Property Name="FileInfo[114].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[115].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[115].FileName" Type="Str">Cursor Point Color Read.vi</Property>
				<Property Name="FileInfo[115].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Color Read.vi</Property>
				<Property Name="FileInfo[115].Type" Type="Int">3</Property>
				<Property Name="FileInfo[115].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[116].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[116].FileName" Type="Str">Cursor Point Color Write.vi</Property>
				<Property Name="FileInfo[116].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Color Write.vi</Property>
				<Property Name="FileInfo[116].Type" Type="Int">3</Property>
				<Property Name="FileInfo[116].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[117].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[117].FileName" Type="Str">Cursor Point Size Read.vi</Property>
				<Property Name="FileInfo[117].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Size Read.vi</Property>
				<Property Name="FileInfo[117].Type" Type="Int">3</Property>
				<Property Name="FileInfo[117].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[118].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[118].FileName" Type="Str">Cursor Point Size Write.vi</Property>
				<Property Name="FileInfo[118].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Size Write.vi</Property>
				<Property Name="FileInfo[118].Type" Type="Int">3</Property>
				<Property Name="FileInfo[118].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[119].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[119].FileName" Type="Str">Cursor Point Style Read.vi</Property>
				<Property Name="FileInfo[119].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Style Read.vi</Property>
				<Property Name="FileInfo[119].Type" Type="Int">3</Property>
				<Property Name="FileInfo[119].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[12].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[12].FileName" Type="Str">View Direction Write.vi</Property>
				<Property Name="FileInfo[12].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Direction Write.vi</Property>
				<Property Name="FileInfo[12].Type" Type="Int">3</Property>
				<Property Name="FileInfo[12].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[120].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[120].FileName" Type="Str">Cursor Point Style Write.vi</Property>
				<Property Name="FileInfo[120].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Point Style Write.vi</Property>
				<Property Name="FileInfo[120].Type" Type="Int">3</Property>
				<Property Name="FileInfo[120].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[121].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[121].FileName" Type="Str">Cursor Line Color Read.vi</Property>
				<Property Name="FileInfo[121].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Color Read.vi</Property>
				<Property Name="FileInfo[121].Type" Type="Int">3</Property>
				<Property Name="FileInfo[121].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[122].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[122].FileName" Type="Str">Cursor Line Color Write.vi</Property>
				<Property Name="FileInfo[122].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Color Write.vi</Property>
				<Property Name="FileInfo[122].Type" Type="Int">3</Property>
				<Property Name="FileInfo[122].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[123].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[123].FileName" Type="Str">Cursor Line Width Read.vi</Property>
				<Property Name="FileInfo[123].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Width Read.vi</Property>
				<Property Name="FileInfo[123].Type" Type="Int">3</Property>
				<Property Name="FileInfo[123].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[124].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[124].FileName" Type="Str">Cursor Line Width Write.vi</Property>
				<Property Name="FileInfo[124].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Width Write.vi</Property>
				<Property Name="FileInfo[124].Type" Type="Int">3</Property>
				<Property Name="FileInfo[124].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[125].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[125].FileName" Type="Str">Cursor Line Style Read.vi</Property>
				<Property Name="FileInfo[125].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Style Read.vi</Property>
				<Property Name="FileInfo[125].Type" Type="Int">3</Property>
				<Property Name="FileInfo[125].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[126].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[126].FileName" Type="Str">Cursor Line Style Write.vi</Property>
				<Property Name="FileInfo[126].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Line Style Write.vi</Property>
				<Property Name="FileInfo[126].Type" Type="Int">3</Property>
				<Property Name="FileInfo[126].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[127].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[127].FileName" Type="Str">Cursor Plane Color Read.vi</Property>
				<Property Name="FileInfo[127].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane Color Read.vi</Property>
				<Property Name="FileInfo[127].Type" Type="Int">3</Property>
				<Property Name="FileInfo[127].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[128].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[128].FileName" Type="Str">Cursor Plane Color Write.vi</Property>
				<Property Name="FileInfo[128].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane Color Write.vi</Property>
				<Property Name="FileInfo[128].Type" Type="Int">3</Property>
				<Property Name="FileInfo[128].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[129].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[129].FileName" Type="Str">Cursor Plane Opacity Read.vi</Property>
				<Property Name="FileInfo[129].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane Opacity Read.vi</Property>
				<Property Name="FileInfo[129].Type" Type="Int">3</Property>
				<Property Name="FileInfo[129].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[13].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[13].FileName" Type="Str">View Latitude Read.vi</Property>
				<Property Name="FileInfo[13].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Latitude Read.vi</Property>
				<Property Name="FileInfo[13].Type" Type="Int">3</Property>
				<Property Name="FileInfo[13].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[130].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[130].FileName" Type="Str">Cursor Plane Opacity Write.vi</Property>
				<Property Name="FileInfo[130].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane Opacity Write.vi</Property>
				<Property Name="FileInfo[130].Type" Type="Int">3</Property>
				<Property Name="FileInfo[130].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[131].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[131].FileName" Type="Str">Cursor Plane XY Visible Read.vi</Property>
				<Property Name="FileInfo[131].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane XY Visible Read.vi</Property>
				<Property Name="FileInfo[131].Type" Type="Int">3</Property>
				<Property Name="FileInfo[131].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[132].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[132].FileName" Type="Str">Cursor Plane XY Visible Write.vi</Property>
				<Property Name="FileInfo[132].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane XY Visible Write.vi</Property>
				<Property Name="FileInfo[132].Type" Type="Int">3</Property>
				<Property Name="FileInfo[132].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[133].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[133].FileName" Type="Str">Cursor Plane XZ Visible Read.vi</Property>
				<Property Name="FileInfo[133].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane XZ Visible Read.vi</Property>
				<Property Name="FileInfo[133].Type" Type="Int">3</Property>
				<Property Name="FileInfo[133].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[134].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[134].FileName" Type="Str">Cursor Plane XZ Visible Write.vi</Property>
				<Property Name="FileInfo[134].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane XZ Visible Write.vi</Property>
				<Property Name="FileInfo[134].Type" Type="Int">3</Property>
				<Property Name="FileInfo[134].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[135].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[135].FileName" Type="Str">Cursor Plane YZ Visible Read.vi</Property>
				<Property Name="FileInfo[135].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane YZ Visible Read.vi</Property>
				<Property Name="FileInfo[135].Type" Type="Int">3</Property>
				<Property Name="FileInfo[135].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[136].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[136].FileName" Type="Str">Cursor Plane YZ Visible Write.vi</Property>
				<Property Name="FileInfo[136].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Plane YZ Visible Write.vi</Property>
				<Property Name="FileInfo[136].Type" Type="Int">3</Property>
				<Property Name="FileInfo[136].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[137].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[137].FileName" Type="Str">Cursor Name Read.vi</Property>
				<Property Name="FileInfo[137].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Name Read.vi</Property>
				<Property Name="FileInfo[137].Type" Type="Int">3</Property>
				<Property Name="FileInfo[137].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[138].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[138].FileName" Type="Str">Cursor Name Write.vi</Property>
				<Property Name="FileInfo[138].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Name Write.vi</Property>
				<Property Name="FileInfo[138].Type" Type="Int">3</Property>
				<Property Name="FileInfo[138].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[139].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[139].FileName" Type="Str">Cursor Text Color Read.vi</Property>
				<Property Name="FileInfo[139].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Color Read.vi</Property>
				<Property Name="FileInfo[139].Type" Type="Int">3</Property>
				<Property Name="FileInfo[139].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[14].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[14].FileName" Type="Str">View Latitude Write.vi</Property>
				<Property Name="FileInfo[14].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Latitude Write.vi</Property>
				<Property Name="FileInfo[14].Type" Type="Int">3</Property>
				<Property Name="FileInfo[14].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[140].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[140].FileName" Type="Str">Cursor Text Color Write.vi</Property>
				<Property Name="FileInfo[140].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Color Write.vi</Property>
				<Property Name="FileInfo[140].Type" Type="Int">3</Property>
				<Property Name="FileInfo[140].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[141].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[141].FileName" Type="Str">Cursor Text Size Read.vi</Property>
				<Property Name="FileInfo[141].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Size Read.vi</Property>
				<Property Name="FileInfo[141].Type" Type="Int">3</Property>
				<Property Name="FileInfo[141].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[142].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[142].FileName" Type="Str">Cursor Text Size Write.vi</Property>
				<Property Name="FileInfo[142].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Size Write.vi</Property>
				<Property Name="FileInfo[142].Type" Type="Int">3</Property>
				<Property Name="FileInfo[142].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[143].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[143].FileName" Type="Str">Cursor Show Position Read.vi</Property>
				<Property Name="FileInfo[143].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Show Position Read.vi</Property>
				<Property Name="FileInfo[143].Type" Type="Int">3</Property>
				<Property Name="FileInfo[143].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[144].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[144].FileName" Type="Str">Cursor Show Position Write.vi</Property>
				<Property Name="FileInfo[144].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Show Position Write.vi</Property>
				<Property Name="FileInfo[144].Type" Type="Int">3</Property>
				<Property Name="FileInfo[144].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[145].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[145].FileName" Type="Str">Cursor Show Name Read.vi</Property>
				<Property Name="FileInfo[145].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Show Name Read.vi</Property>
				<Property Name="FileInfo[145].Type" Type="Int">3</Property>
				<Property Name="FileInfo[145].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[146].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[146].FileName" Type="Str">Cursor Show Name Write.vi</Property>
				<Property Name="FileInfo[146].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Show Name Write.vi</Property>
				<Property Name="FileInfo[146].Type" Type="Int">3</Property>
				<Property Name="FileInfo[146].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[147].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[147].FileName" Type="Str">Cursor Text Background Color Read.vi</Property>
				<Property Name="FileInfo[147].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Background Color Read.vi</Property>
				<Property Name="FileInfo[147].Type" Type="Int">3</Property>
				<Property Name="FileInfo[147].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[148].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[148].FileName" Type="Str">Cursor Text Background Color Write.vi</Property>
				<Property Name="FileInfo[148].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Background Color Write.vi</Property>
				<Property Name="FileInfo[148].Type" Type="Int">3</Property>
				<Property Name="FileInfo[148].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[149].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[149].FileName" Type="Str">Cursor Text Background Opacity Read.vi</Property>
				<Property Name="FileInfo[149].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Background Opacity Read.vi</Property>
				<Property Name="FileInfo[149].Type" Type="Int">3</Property>
				<Property Name="FileInfo[149].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[15].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[15].FileName" Type="Str">View Longitude Read.vi</Property>
				<Property Name="FileInfo[15].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Longitude Read.vi</Property>
				<Property Name="FileInfo[15].Type" Type="Int">3</Property>
				<Property Name="FileInfo[15].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[150].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[150].FileName" Type="Str">Cursor Text Background Opacity Write.vi</Property>
				<Property Name="FileInfo[150].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor Text Background Opacity Write.vi</Property>
				<Property Name="FileInfo[150].Type" Type="Int">3</Property>
				<Property Name="FileInfo[150].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[151].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[151].FileName" Type="Str">Cursor List Read.vi</Property>
				<Property Name="FileInfo[151].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor List Read.vi</Property>
				<Property Name="FileInfo[151].Type" Type="Int">3</Property>
				<Property Name="FileInfo[151].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[152].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[152].FileName" Type="Str">Cursor List Write.vi</Property>
				<Property Name="FileInfo[152].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Cursor List Write.vi</Property>
				<Property Name="FileInfo[152].Type" Type="Int">3</Property>
				<Property Name="FileInfo[152].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[153].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[153].FileName" Type="Str">Read Color Cluster.vi</Property>
				<Property Name="FileInfo[153].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Read Color Cluster.vi</Property>
				<Property Name="FileInfo[153].Type" Type="Int">3</Property>
				<Property Name="FileInfo[153].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[154].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[154].FileName" Type="Str">Image to Printer.vi</Property>
				<Property Name="FileInfo[154].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Menu/Image to Printer.vi</Property>
				<Property Name="FileInfo[154].Type" Type="Int">3</Property>
				<Property Name="FileInfo[154].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[155].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[155].FileName" Type="Str">Image to File.vi</Property>
				<Property Name="FileInfo[155].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Menu/Image to File.vi</Property>
				<Property Name="FileInfo[155].Type" Type="Int">3</Property>
				<Property Name="FileInfo[155].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[156].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[156].FileName" Type="Str">Property Dialog.vi</Property>
				<Property Name="FileInfo[156].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property Page/Property Dialog.vi</Property>
				<Property Name="FileInfo[156].Type" Type="Int">3</Property>
				<Property Name="FileInfo[156].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[157].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[157].FileName" Type="Str">Backup Plot Name.vi</Property>
				<Property Name="FileInfo[157].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Backup Plot Name.vi</Property>
				<Property Name="FileInfo[157].Type" Type="Int">3</Property>
				<Property Name="FileInfo[157].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[158].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[158].FileName" Type="Str">Get Coordinates.vi</Property>
				<Property Name="FileInfo[158].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Get Coordinates.vi</Property>
				<Property Name="FileInfo[158].Type" Type="Int">3</Property>
				<Property Name="FileInfo[158].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[159].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[159].FileName" Type="Str">Create Plot Object.vi</Property>
				<Property Name="FileInfo[159].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Protected/Plot Object/Create Plot Object.vi</Property>
				<Property Name="FileInfo[159].Type" Type="Int">3</Property>
				<Property Name="FileInfo[159].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[16].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[16].FileName" Type="Str">View Longitude Write.vi</Property>
				<Property Name="FileInfo[16].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Longitude Write.vi</Property>
				<Property Name="FileInfo[16].Type" Type="Int">3</Property>
				<Property Name="FileInfo[16].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[160].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[160].FileName" Type="Str">Create Clip Plane.vi</Property>
				<Property Name="FileInfo[160].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Protected/Plot Object/Create Clip Plane.vi</Property>
				<Property Name="FileInfo[160].Type" Type="Int">3</Property>
				<Property Name="FileInfo[160].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[161].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[161].FileName" Type="Str">Create Fast Draw Object.vi</Property>
				<Property Name="FileInfo[161].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Protected/Fast Draw Object/Create Fast Draw Object.vi</Property>
				<Property Name="FileInfo[161].Type" Type="Int">3</Property>
				<Property Name="FileInfo[161].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[162].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[162].FileName" Type="Str">Set Variant Property.vi</Property>
				<Property Name="FileInfo[162].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Protected/Set Variant Property.vi</Property>
				<Property Name="FileInfo[162].Type" Type="Int">3</Property>
				<Property Name="FileInfo[162].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[163].DirTag" Type="Str">{0051B67B-C11C-4737-8852-03A772D3AF14}</Property>
				<Property Name="FileInfo[163].FileName" Type="Str">Calculate Vertex Array.vi</Property>
				<Property Name="FileInfo[163].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot Datatype.lvclass/Protected/Create Plot Data/Calculate Vertex Array.vi</Property>
				<Property Name="FileInfo[163].Type" Type="Int">3</Property>
				<Property Name="FileInfo[163].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[164].DirTag" Type="Str">{0051B67B-C11C-4737-8852-03A772D3AF14}</Property>
				<Property Name="FileInfo[164].FileName" Type="Str">Calculate Color Index Array.vi</Property>
				<Property Name="FileInfo[164].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot Datatype.lvclass/Protected/Create Plot Data/Calculate Color Index Array.vi</Property>
				<Property Name="FileInfo[164].Type" Type="Int">3</Property>
				<Property Name="FileInfo[164].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[165].DirTag" Type="Str">{0051B67B-C11C-4737-8852-03A772D3AF14}</Property>
				<Property Name="FileInfo[165].FileName" Type="Str">Get Plot ID.vi</Property>
				<Property Name="FileInfo[165].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot Datatype.lvclass/Protected/Create Plot Data/Get Plot ID.vi</Property>
				<Property Name="FileInfo[165].Type" Type="Int">3</Property>
				<Property Name="FileInfo[165].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[166].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[166].FileName" Type="Str">Fast Draw Read.vi</Property>
				<Property Name="FileInfo[166].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Fast Draw/Fast Draw Read.vi</Property>
				<Property Name="FileInfo[166].Type" Type="Int">3</Property>
				<Property Name="FileInfo[166].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[167].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[167].FileName" Type="Str">Read Color Cluster.vi</Property>
				<Property Name="FileInfo[167].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Public/Property/Read Color Cluster.vi</Property>
				<Property Name="FileInfo[167].Type" Type="Int">3</Property>
				<Property Name="FileInfo[167].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[168].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[168].FileName" Type="Str">Property Dialog.vi</Property>
				<Property Name="FileInfo[168].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Public/Property Page/Property Dialog.vi</Property>
				<Property Name="FileInfo[168].Type" Type="Int">3</Property>
				<Property Name="FileInfo[168].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[169].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[169].FileName" Type="Str">Backup Plot Name.vi</Property>
				<Property Name="FileInfo[169].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Public/Backup Plot Name.vi</Property>
				<Property Name="FileInfo[169].Type" Type="Int">3</Property>
				<Property Name="FileInfo[169].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[17].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[17].FileName" Type="Str">View Distance Read.vi</Property>
				<Property Name="FileInfo[17].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Distance Read.vi</Property>
				<Property Name="FileInfo[17].Type" Type="Int">3</Property>
				<Property Name="FileInfo[17].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[170].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[170].FileName" Type="Str">Get Coordinates.vi</Property>
				<Property Name="FileInfo[170].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Public/Get Coordinates.vi</Property>
				<Property Name="FileInfo[170].Type" Type="Int">3</Property>
				<Property Name="FileInfo[170].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[171].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[171].FileName" Type="Str">Create Plot Object.vi</Property>
				<Property Name="FileInfo[171].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Protected/Plot Object/Create Plot Object.vi</Property>
				<Property Name="FileInfo[171].Type" Type="Int">3</Property>
				<Property Name="FileInfo[171].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[172].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[172].FileName" Type="Str">Create Fast Draw Object.vi</Property>
				<Property Name="FileInfo[172].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Protected/Fast Draw Object/Create Fast Draw Object.vi</Property>
				<Property Name="FileInfo[172].Type" Type="Int">3</Property>
				<Property Name="FileInfo[172].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[173].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[173].FileName" Type="Str">Set Variant Property.vi</Property>
				<Property Name="FileInfo[173].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.lvclass/Protected/Set Variant Property.vi</Property>
				<Property Name="FileInfo[173].Type" Type="Int">3</Property>
				<Property Name="FileInfo[173].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[174].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[174].FileName" Type="Str">Fast Draw Write.vi</Property>
				<Property Name="FileInfo[174].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Fast Draw/Fast Draw Write.vi</Property>
				<Property Name="FileInfo[174].Type" Type="Int">3</Property>
				<Property Name="FileInfo[174].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[175].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[175].FileName" Type="Str">Clip Data Read.vi</Property>
				<Property Name="FileInfo[175].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Clip Data/Clip Data Read.vi</Property>
				<Property Name="FileInfo[175].Type" Type="Int">3</Property>
				<Property Name="FileInfo[175].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[176].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[176].FileName" Type="Str">Clip Data Write.vi</Property>
				<Property Name="FileInfo[176].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Clip Data/Clip Data Write.vi</Property>
				<Property Name="FileInfo[176].Type" Type="Int">3</Property>
				<Property Name="FileInfo[176].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[177].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[177].FileName" Type="Str">Projection Mode Read.vi</Property>
				<Property Name="FileInfo[177].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Projection Mode/Projection Mode Read.vi</Property>
				<Property Name="FileInfo[177].Type" Type="Int">3</Property>
				<Property Name="FileInfo[177].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[178].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[178].FileName" Type="Str">Projection Mode Write.vi</Property>
				<Property Name="FileInfo[178].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/Projection Mode/Projection Mode Write.vi</Property>
				<Property Name="FileInfo[178].Type" Type="Int">3</Property>
				<Property Name="FileInfo[178].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[179].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[179].FileName" Type="Str">View Direction Read.vi</Property>
				<Property Name="FileInfo[179].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Direction/View Direction Read.vi</Property>
				<Property Name="FileInfo[179].Type" Type="Int">3</Property>
				<Property Name="FileInfo[179].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[18].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[18].FileName" Type="Str">View Distance Write.vi</Property>
				<Property Name="FileInfo[18].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/View Distance Write.vi</Property>
				<Property Name="FileInfo[18].Type" Type="Int">3</Property>
				<Property Name="FileInfo[18].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[180].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[180].FileName" Type="Str">View Direction Write.vi</Property>
				<Property Name="FileInfo[180].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Direction/View Direction Write.vi</Property>
				<Property Name="FileInfo[180].Type" Type="Int">3</Property>
				<Property Name="FileInfo[180].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[181].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[181].FileName" Type="Str">View Latitude Read.vi</Property>
				<Property Name="FileInfo[181].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Latitude/View Latitude Read.vi</Property>
				<Property Name="FileInfo[181].Type" Type="Int">3</Property>
				<Property Name="FileInfo[181].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[182].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[182].FileName" Type="Str">View Latitude Write.vi</Property>
				<Property Name="FileInfo[182].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Latitude/View Latitude Write.vi</Property>
				<Property Name="FileInfo[182].Type" Type="Int">3</Property>
				<Property Name="FileInfo[182].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[183].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[183].FileName" Type="Str">View Longitude Read.vi</Property>
				<Property Name="FileInfo[183].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Longitude/View Longitude Read.vi</Property>
				<Property Name="FileInfo[183].Type" Type="Int">3</Property>
				<Property Name="FileInfo[183].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[184].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[184].FileName" Type="Str">View Longitude Write.vi</Property>
				<Property Name="FileInfo[184].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Longitude/View Longitude Write.vi</Property>
				<Property Name="FileInfo[184].Type" Type="Int">3</Property>
				<Property Name="FileInfo[184].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[185].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[185].FileName" Type="Str">View Distance Read.vi</Property>
				<Property Name="FileInfo[185].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Distance/View Distance Read.vi</Property>
				<Property Name="FileInfo[185].Type" Type="Int">3</Property>
				<Property Name="FileInfo[185].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[186].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[186].FileName" Type="Str">View Distance Write.vi</Property>
				<Property Name="FileInfo[186].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/View Distance/View Distance Write.vi</Property>
				<Property Name="FileInfo[186].Type" Type="Int">3</Property>
				<Property Name="FileInfo[186].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[187].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[187].FileName" Type="Str">Grid XY Plane Visible Read.vi</Property>
				<Property Name="FileInfo[187].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/XY Plane Visible/Grid XY Plane Visible Read.vi</Property>
				<Property Name="FileInfo[187].Type" Type="Int">3</Property>
				<Property Name="FileInfo[187].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[188].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[188].FileName" Type="Str">Grid XY Plane Visible Write.vi</Property>
				<Property Name="FileInfo[188].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/XY Plane Visible/Grid XY Plane Visible Write.vi</Property>
				<Property Name="FileInfo[188].Type" Type="Int">3</Property>
				<Property Name="FileInfo[188].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[189].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[189].FileName" Type="Str">Grid YZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[189].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/YZ Plane Visible/Grid YZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[189].Type" Type="Int">3</Property>
				<Property Name="FileInfo[189].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[19].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[19].FileName" Type="Str">Grid XY Plane Visible Read.vi</Property>
				<Property Name="FileInfo[19].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid XY Plane Visible Read.vi</Property>
				<Property Name="FileInfo[19].Type" Type="Int">3</Property>
				<Property Name="FileInfo[19].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[190].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[190].FileName" Type="Str">Grid YZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[190].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/YZ Plane Visible/Grid YZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[190].Type" Type="Int">3</Property>
				<Property Name="FileInfo[190].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[191].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[191].FileName" Type="Str">Grid XZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[191].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/XZ Plane Visible/Grid XZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[191].Type" Type="Int">3</Property>
				<Property Name="FileInfo[191].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[192].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[192].FileName" Type="Str">Grid XZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[192].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Graph/XZ Plane Visible/Grid XZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[192].Type" Type="Int">3</Property>
				<Property Name="FileInfo[192].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[193].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[193].FileName" Type="Str">Enable Lighting Read.vi</Property>
				<Property Name="FileInfo[193].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Enable Lighting/Enable Lighting Read.vi</Property>
				<Property Name="FileInfo[193].Type" Type="Int">3</Property>
				<Property Name="FileInfo[193].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[194].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[194].FileName" Type="Str">Enable Lighting Write.vi</Property>
				<Property Name="FileInfo[194].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Enable Lighting/Enable Lighting Write.vi</Property>
				<Property Name="FileInfo[194].Type" Type="Int">3</Property>
				<Property Name="FileInfo[194].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[195].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[195].FileName" Type="Str">Active Light Read.vi</Property>
				<Property Name="FileInfo[195].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Active Light/Active Light Read.vi</Property>
				<Property Name="FileInfo[195].Type" Type="Int">3</Property>
				<Property Name="FileInfo[195].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[196].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[196].FileName" Type="Str">Active Light Write.vi</Property>
				<Property Name="FileInfo[196].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Active Light/Active Light Write.vi</Property>
				<Property Name="FileInfo[196].Type" Type="Int">3</Property>
				<Property Name="FileInfo[196].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[197].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[197].FileName" Type="Str">Enable Light Read.vi</Property>
				<Property Name="FileInfo[197].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Enable Light/Enable Light Read.vi</Property>
				<Property Name="FileInfo[197].Type" Type="Int">3</Property>
				<Property Name="FileInfo[197].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[198].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[198].FileName" Type="Str">Enable Light Write.vi</Property>
				<Property Name="FileInfo[198].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Enable Light/Enable Light Write.vi</Property>
				<Property Name="FileInfo[198].Type" Type="Int">3</Property>
				<Property Name="FileInfo[198].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[199].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[199].FileName" Type="Str">Light Color Read.vi</Property>
				<Property Name="FileInfo[199].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Color/Light Color Read.vi</Property>
				<Property Name="FileInfo[199].Type" Type="Int">3</Property>
				<Property Name="FileInfo[199].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[2].DirTag" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="FileInfo[2].FileName" Type="Str">AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[2].FileTag" Type="Str">{95AB525D-9CC0-48EA-84D7-D36AB922FE00}</Property>
				<Property Name="FileInfo[2].Type" Type="Int">3</Property>
				<Property Name="FileInfo[2].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[20].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[20].FileName" Type="Str">Grid XY Plane Visible Write.vi</Property>
				<Property Name="FileInfo[20].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid XY Plane Visible Write.vi</Property>
				<Property Name="FileInfo[20].Type" Type="Int">3</Property>
				<Property Name="FileInfo[20].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[200].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[200].FileName" Type="Str">Light Color Write.vi</Property>
				<Property Name="FileInfo[200].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Color/Light Color Write.vi</Property>
				<Property Name="FileInfo[200].Type" Type="Int">3</Property>
				<Property Name="FileInfo[200].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[201].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[201].FileName" Type="Str">Light Latitude Read.vi</Property>
				<Property Name="FileInfo[201].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Latitude/Light Latitude Read.vi</Property>
				<Property Name="FileInfo[201].Type" Type="Int">3</Property>
				<Property Name="FileInfo[201].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[202].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[202].FileName" Type="Str">Light Latitude Write.vi</Property>
				<Property Name="FileInfo[202].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Latitude/Light Latitude Write.vi</Property>
				<Property Name="FileInfo[202].Type" Type="Int">3</Property>
				<Property Name="FileInfo[202].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[203].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[203].FileName" Type="Str">Light Longitude Read.vi</Property>
				<Property Name="FileInfo[203].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Longitude/Light Longitude Read.vi</Property>
				<Property Name="FileInfo[203].Type" Type="Int">3</Property>
				<Property Name="FileInfo[203].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[204].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[204].FileName" Type="Str">Light Longitude Write.vi</Property>
				<Property Name="FileInfo[204].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Longitude/Light Longitude Write.vi</Property>
				<Property Name="FileInfo[204].Type" Type="Int">3</Property>
				<Property Name="FileInfo[204].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[205].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[205].FileName" Type="Str">Light Distance Read.vi</Property>
				<Property Name="FileInfo[205].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Distance/Light Distance Read.vi</Property>
				<Property Name="FileInfo[205].Type" Type="Int">3</Property>
				<Property Name="FileInfo[205].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[206].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[206].FileName" Type="Str">Light Distance Write.vi</Property>
				<Property Name="FileInfo[206].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Distance/Light Distance Write.vi</Property>
				<Property Name="FileInfo[206].Type" Type="Int">3</Property>
				<Property Name="FileInfo[206].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[207].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[207].FileName" Type="Str">Light Attenuation Read.vi</Property>
				<Property Name="FileInfo[207].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Attenuation/Light Attenuation Read.vi</Property>
				<Property Name="FileInfo[207].Type" Type="Int">3</Property>
				<Property Name="FileInfo[207].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[208].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[208].FileName" Type="Str">Light Attenuation Write.vi</Property>
				<Property Name="FileInfo[208].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Light/Light Attenuation/Light Attenuation Write.vi</Property>
				<Property Name="FileInfo[208].Type" Type="Int">3</Property>
				<Property Name="FileInfo[208].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[209].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[209].FileName" Type="Str">Active Axis Read.vi</Property>
				<Property Name="FileInfo[209].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Active Axis/Active Axis Read.vi</Property>
				<Property Name="FileInfo[209].Type" Type="Int">3</Property>
				<Property Name="FileInfo[209].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[21].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[21].FileName" Type="Str">Grid YZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[21].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid YZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[21].Type" Type="Int">3</Property>
				<Property Name="FileInfo[21].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[210].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[210].FileName" Type="Str">Active Axis Write.vi</Property>
				<Property Name="FileInfo[210].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Active Axis/Active Axis Write.vi</Property>
				<Property Name="FileInfo[210].Type" Type="Int">3</Property>
				<Property Name="FileInfo[210].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[211].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[211].FileName" Type="Str">Name Label Color Read.vi</Property>
				<Property Name="FileInfo[211].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Color/Name Label Color Read.vi</Property>
				<Property Name="FileInfo[211].Type" Type="Int">3</Property>
				<Property Name="FileInfo[211].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[212].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[212].FileName" Type="Str">Name Label Color Write.vi</Property>
				<Property Name="FileInfo[212].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Color/Name Label Color Write.vi</Property>
				<Property Name="FileInfo[212].Type" Type="Int">3</Property>
				<Property Name="FileInfo[212].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[213].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[213].FileName" Type="Str">Name Label Size Read.vi</Property>
				<Property Name="FileInfo[213].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Size/Name Label Size Read.vi</Property>
				<Property Name="FileInfo[213].Type" Type="Int">3</Property>
				<Property Name="FileInfo[213].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[214].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[214].FileName" Type="Str">Name Label Size Write.vi</Property>
				<Property Name="FileInfo[214].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Size/Name Label Size Write.vi</Property>
				<Property Name="FileInfo[214].Type" Type="Int">3</Property>
				<Property Name="FileInfo[214].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[215].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[215].FileName" Type="Str">Name Label Text Read.vi</Property>
				<Property Name="FileInfo[215].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Text/Name Label Text Read.vi</Property>
				<Property Name="FileInfo[215].Type" Type="Int">3</Property>
				<Property Name="FileInfo[215].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[216].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[216].FileName" Type="Str">Name Label Text Write.vi</Property>
				<Property Name="FileInfo[216].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Text/Name Label Text Write.vi</Property>
				<Property Name="FileInfo[216].Type" Type="Int">3</Property>
				<Property Name="FileInfo[216].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[217].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[217].FileName" Type="Str">Name Label Normal Visible Read.vi</Property>
				<Property Name="FileInfo[217].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Normal Visible/Name Label Normal Visible Read.vi</Property>
				<Property Name="FileInfo[217].Type" Type="Int">3</Property>
				<Property Name="FileInfo[217].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[218].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[218].FileName" Type="Str">Name Label Normal Visible Write.vi</Property>
				<Property Name="FileInfo[218].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Normal Visible/Name Label Normal Visible Write.vi</Property>
				<Property Name="FileInfo[218].Type" Type="Int">3</Property>
				<Property Name="FileInfo[218].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[219].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[219].FileName" Type="Str">Name Label Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[219].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Opposite Visible/Name Label Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[219].Type" Type="Int">3</Property>
				<Property Name="FileInfo[219].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[22].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[22].FileName" Type="Str">Grid YZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[22].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid YZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[22].Type" Type="Int">3</Property>
				<Property Name="FileInfo[22].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[220].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[220].FileName" Type="Str">Name Label Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[220].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Name Label Opposite Visible/Name Label Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[220].Type" Type="Int">3</Property>
				<Property Name="FileInfo[220].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[221].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[221].FileName" Type="Str">Marker Color Read.vi</Property>
				<Property Name="FileInfo[221].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Color/Marker Color Read.vi</Property>
				<Property Name="FileInfo[221].Type" Type="Int">3</Property>
				<Property Name="FileInfo[221].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[222].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[222].FileName" Type="Str">Marker Color Write.vi</Property>
				<Property Name="FileInfo[222].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Color/Marker Color Write.vi</Property>
				<Property Name="FileInfo[222].Type" Type="Int">3</Property>
				<Property Name="FileInfo[222].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[223].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[223].FileName" Type="Str">Marker Size Read.vi</Property>
				<Property Name="FileInfo[223].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Size/Marker Size Read.vi</Property>
				<Property Name="FileInfo[223].Type" Type="Int">3</Property>
				<Property Name="FileInfo[223].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[224].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[224].FileName" Type="Str">Marker Size Write.vi</Property>
				<Property Name="FileInfo[224].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Size/Marker Size Write.vi</Property>
				<Property Name="FileInfo[224].Type" Type="Int">3</Property>
				<Property Name="FileInfo[224].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[225].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[225].FileName" Type="Str">Marker Normal Visible Read.vi</Property>
				<Property Name="FileInfo[225].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Normal Visible/Marker Normal Visible Read.vi</Property>
				<Property Name="FileInfo[225].Type" Type="Int">3</Property>
				<Property Name="FileInfo[225].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[226].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[226].FileName" Type="Str">Marker Normal Visible Write.vi</Property>
				<Property Name="FileInfo[226].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Normal Visible/Marker Normal Visible Write.vi</Property>
				<Property Name="FileInfo[226].Type" Type="Int">3</Property>
				<Property Name="FileInfo[226].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[227].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[227].FileName" Type="Str">Marker Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[227].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Opposite Visible/Marker Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[227].Type" Type="Int">3</Property>
				<Property Name="FileInfo[227].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[228].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[228].FileName" Type="Str">Marker Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[228].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Marker Opposite Visible/Marker Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[228].Type" Type="Int">3</Property>
				<Property Name="FileInfo[228].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[229].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[229].FileName" Type="Str">Range Maximum Read.vi</Property>
				<Property Name="FileInfo[229].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Maximum/Range Maximum Read.vi</Property>
				<Property Name="FileInfo[229].Type" Type="Int">3</Property>
				<Property Name="FileInfo[229].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[23].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[23].FileName" Type="Str">Grid XZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[23].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid XZ Plane Visible Read.vi</Property>
				<Property Name="FileInfo[23].Type" Type="Int">3</Property>
				<Property Name="FileInfo[23].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[230].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[230].FileName" Type="Str">Range Maximum Write.vi</Property>
				<Property Name="FileInfo[230].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Maximum/Range Maximum Write.vi</Property>
				<Property Name="FileInfo[230].Type" Type="Int">3</Property>
				<Property Name="FileInfo[230].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[231].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[231].FileName" Type="Str">Range Minimum Read.vi</Property>
				<Property Name="FileInfo[231].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Minimum/Range Minimum Read.vi</Property>
				<Property Name="FileInfo[231].Type" Type="Int">3</Property>
				<Property Name="FileInfo[231].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[232].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[232].FileName" Type="Str">Range Minimum Write.vi</Property>
				<Property Name="FileInfo[232].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Minimum/Range Minimum Write.vi</Property>
				<Property Name="FileInfo[232].Type" Type="Int">3</Property>
				<Property Name="FileInfo[232].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[233].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[233].FileName" Type="Str">Range Inverted Read.vi</Property>
				<Property Name="FileInfo[233].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Inverted/Range Inverted Read.vi</Property>
				<Property Name="FileInfo[233].Type" Type="Int">3</Property>
				<Property Name="FileInfo[233].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[234].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[234].FileName" Type="Str">Range Inverted Write.vi</Property>
				<Property Name="FileInfo[234].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Inverted/Range Inverted Write.vi</Property>
				<Property Name="FileInfo[234].Type" Type="Int">3</Property>
				<Property Name="FileInfo[234].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[235].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[235].FileName" Type="Str">Range Logarithm Read.vi</Property>
				<Property Name="FileInfo[235].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Logarithm/Range Logarithm Read.vi</Property>
				<Property Name="FileInfo[235].Type" Type="Int">3</Property>
				<Property Name="FileInfo[235].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[236].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[236].FileName" Type="Str">Range Logarithm Write.vi</Property>
				<Property Name="FileInfo[236].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Range Logarithm/Range Logarithm Write.vi</Property>
				<Property Name="FileInfo[236].Type" Type="Int">3</Property>
				<Property Name="FileInfo[236].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[237].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[237].FileName" Type="Str">Grid Major Color Read.vi</Property>
				<Property Name="FileInfo[237].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Major Color/Grid Major Color Read.vi</Property>
				<Property Name="FileInfo[237].Type" Type="Int">3</Property>
				<Property Name="FileInfo[237].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[238].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[238].FileName" Type="Str">Grid Major Color Write.vi</Property>
				<Property Name="FileInfo[238].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Major Color/Grid Major Color Write.vi</Property>
				<Property Name="FileInfo[238].Type" Type="Int">3</Property>
				<Property Name="FileInfo[238].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[239].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[239].FileName" Type="Str">Grid Major Visible Read.vi</Property>
				<Property Name="FileInfo[239].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Major Visible/Grid Major Visible Read.vi</Property>
				<Property Name="FileInfo[239].Type" Type="Int">3</Property>
				<Property Name="FileInfo[239].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[24].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[24].FileName" Type="Str">Grid XZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[24].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Grid XZ Plane Visible Write.vi</Property>
				<Property Name="FileInfo[24].Type" Type="Int">3</Property>
				<Property Name="FileInfo[24].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[240].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[240].FileName" Type="Str">Grid Major Visible Write.vi</Property>
				<Property Name="FileInfo[240].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Major Visible/Grid Major Visible Write.vi</Property>
				<Property Name="FileInfo[240].Type" Type="Int">3</Property>
				<Property Name="FileInfo[240].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[241].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[241].FileName" Type="Str">Grid Minor Color Read.vi</Property>
				<Property Name="FileInfo[241].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Minor Color/Grid Minor Color Read.vi</Property>
				<Property Name="FileInfo[241].Type" Type="Int">3</Property>
				<Property Name="FileInfo[241].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[242].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[242].FileName" Type="Str">Grid Minor Color Write.vi</Property>
				<Property Name="FileInfo[242].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Minor Color/Grid Minor Color Write.vi</Property>
				<Property Name="FileInfo[242].Type" Type="Int">3</Property>
				<Property Name="FileInfo[242].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[243].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[243].FileName" Type="Str">Grid Minor Visible Read.vi</Property>
				<Property Name="FileInfo[243].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Minor Visible/Grid Minor Visible Read.vi</Property>
				<Property Name="FileInfo[243].Type" Type="Int">3</Property>
				<Property Name="FileInfo[243].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[244].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[244].FileName" Type="Str">Grid Minor Visible Write.vi</Property>
				<Property Name="FileInfo[244].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Grid Minor Visible/Grid Minor Visible Write.vi</Property>
				<Property Name="FileInfo[244].Type" Type="Int">3</Property>
				<Property Name="FileInfo[244].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[245].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[245].FileName" Type="Str">Tick Major Count Read.vi</Property>
				<Property Name="FileInfo[245].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Count/Tick Major Count Read.vi</Property>
				<Property Name="FileInfo[245].Type" Type="Int">3</Property>
				<Property Name="FileInfo[245].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[246].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[246].FileName" Type="Str">Tick Major Count Write.vi</Property>
				<Property Name="FileInfo[246].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Count/Tick Major Count Write.vi</Property>
				<Property Name="FileInfo[246].Type" Type="Int">3</Property>
				<Property Name="FileInfo[246].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[247].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[247].FileName" Type="Str">Tick Major Color Read.vi</Property>
				<Property Name="FileInfo[247].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Color/Tick Major Color Read.vi</Property>
				<Property Name="FileInfo[247].Type" Type="Int">3</Property>
				<Property Name="FileInfo[247].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[248].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[248].FileName" Type="Str">Tick Major Color Write.vi</Property>
				<Property Name="FileInfo[248].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Color/Tick Major Color Write.vi</Property>
				<Property Name="FileInfo[248].Type" Type="Int">3</Property>
				<Property Name="FileInfo[248].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[249].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[249].FileName" Type="Str">Tick Major Visible Read.vi</Property>
				<Property Name="FileInfo[249].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Visible/Tick Major Visible Read.vi</Property>
				<Property Name="FileInfo[249].Type" Type="Int">3</Property>
				<Property Name="FileInfo[249].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[25].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[25].FileName" Type="Str">Enable Lighting Read.vi</Property>
				<Property Name="FileInfo[25].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Enable Lighting Read.vi</Property>
				<Property Name="FileInfo[25].Type" Type="Int">3</Property>
				<Property Name="FileInfo[25].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[250].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[250].FileName" Type="Str">Tick Major Visible Write.vi</Property>
				<Property Name="FileInfo[250].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Major Visible/Tick Major Visible Write.vi</Property>
				<Property Name="FileInfo[250].Type" Type="Int">3</Property>
				<Property Name="FileInfo[250].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[251].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[251].FileName" Type="Str">Tick Minor Count Read.vi</Property>
				<Property Name="FileInfo[251].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Count/Tick Minor Count Read.vi</Property>
				<Property Name="FileInfo[251].Type" Type="Int">3</Property>
				<Property Name="FileInfo[251].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[252].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[252].FileName" Type="Str">Tick Minor Count Write.vi</Property>
				<Property Name="FileInfo[252].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Count/Tick Minor Count Write.vi</Property>
				<Property Name="FileInfo[252].Type" Type="Int">3</Property>
				<Property Name="FileInfo[252].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[253].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[253].FileName" Type="Str">Tick Minor Color Read.vi</Property>
				<Property Name="FileInfo[253].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Color/Tick Minor Color Read.vi</Property>
				<Property Name="FileInfo[253].Type" Type="Int">3</Property>
				<Property Name="FileInfo[253].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[254].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[254].FileName" Type="Str">Tick Minor Color Write.vi</Property>
				<Property Name="FileInfo[254].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Color/Tick Minor Color Write.vi</Property>
				<Property Name="FileInfo[254].Type" Type="Int">3</Property>
				<Property Name="FileInfo[254].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[255].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[255].FileName" Type="Str">Tick Minor Visible Read.vi</Property>
				<Property Name="FileInfo[255].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Visible/Tick Minor Visible Read.vi</Property>
				<Property Name="FileInfo[255].Type" Type="Int">3</Property>
				<Property Name="FileInfo[255].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[256].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[256].FileName" Type="Str">Tick Minor Visible Write.vi</Property>
				<Property Name="FileInfo[256].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Axis/Tick Minor Visible/Tick Minor Visible Write.vi</Property>
				<Property Name="FileInfo[256].Type" Type="Int">3</Property>
				<Property Name="FileInfo[256].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[257].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[257].FileName" Type="Str">Active Value Pair Read.vi</Property>
				<Property Name="FileInfo[257].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Active Value Pair/Active Value Pair Read.vi</Property>
				<Property Name="FileInfo[257].Type" Type="Int">3</Property>
				<Property Name="FileInfo[257].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[258].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[258].FileName" Type="Str">Active Value Pair Write.vi</Property>
				<Property Name="FileInfo[258].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Active Value Pair/Active Value Pair Write.vi</Property>
				<Property Name="FileInfo[258].Type" Type="Int">3</Property>
				<Property Name="FileInfo[258].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[259].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[259].FileName" Type="Str">Value Pair Label Type Read.vi</Property>
				<Property Name="FileInfo[259].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Label Type/Value Pair Label Type Read.vi</Property>
				<Property Name="FileInfo[259].Type" Type="Int">3</Property>
				<Property Name="FileInfo[259].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[26].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[26].FileName" Type="Str">Enable Lighting Write.vi</Property>
				<Property Name="FileInfo[26].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Enable Lighting Write.vi</Property>
				<Property Name="FileInfo[26].Type" Type="Int">3</Property>
				<Property Name="FileInfo[26].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[260].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[260].FileName" Type="Str">Value Pair Label Type Write.vi</Property>
				<Property Name="FileInfo[260].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Label Type/Value Pair Label Type Write.vi</Property>
				<Property Name="FileInfo[260].Type" Type="Int">3</Property>
				<Property Name="FileInfo[260].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[261].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[261].FileName" Type="Str">Value Pair Tick Visible Read.vi</Property>
				<Property Name="FileInfo[261].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Tick Visible/Value Pair Tick Visible Read.vi</Property>
				<Property Name="FileInfo[261].Type" Type="Int">3</Property>
				<Property Name="FileInfo[261].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[262].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[262].FileName" Type="Str">Value Pair Tick Visible Write.vi</Property>
				<Property Name="FileInfo[262].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Tick Visible/Value Pair Tick Visible Write.vi</Property>
				<Property Name="FileInfo[262].Type" Type="Int">3</Property>
				<Property Name="FileInfo[262].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[263].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[263].FileName" Type="Str">Value Pair Grid Visible Read.vi</Property>
				<Property Name="FileInfo[263].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Grid Visible/Value Pair Grid Visible Read.vi</Property>
				<Property Name="FileInfo[263].Type" Type="Int">3</Property>
				<Property Name="FileInfo[263].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[264].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[264].FileName" Type="Str">Value Pair Grid Visible Write.vi</Property>
				<Property Name="FileInfo[264].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Grid Visible/Value Pair Grid Visible Write.vi</Property>
				<Property Name="FileInfo[264].Type" Type="Int">3</Property>
				<Property Name="FileInfo[264].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[265].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[265].FileName" Type="Str">Value Pair Array Read.vi</Property>
				<Property Name="FileInfo[265].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Array/Value Pair Array Read.vi</Property>
				<Property Name="FileInfo[265].Type" Type="Int">3</Property>
				<Property Name="FileInfo[265].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[266].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[266].FileName" Type="Str">Value Pair Array Write.vi</Property>
				<Property Name="FileInfo[266].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Value Pairs/Value Pair Array/Value Pair Array Write.vi</Property>
				<Property Name="FileInfo[266].Type" Type="Int">3</Property>
				<Property Name="FileInfo[266].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[267].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[267].FileName" Type="Str">Active Cursor Read.vi</Property>
				<Property Name="FileInfo[267].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Active Cursor/Active Cursor Read.vi</Property>
				<Property Name="FileInfo[267].Type" Type="Int">3</Property>
				<Property Name="FileInfo[267].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[268].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[268].FileName" Type="Str">Active Cursor Write.vi</Property>
				<Property Name="FileInfo[268].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Active Cursor/Active Cursor Write.vi</Property>
				<Property Name="FileInfo[268].Type" Type="Int">3</Property>
				<Property Name="FileInfo[268].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[269].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[269].FileName" Type="Str">Cursor Enable Read.vi</Property>
				<Property Name="FileInfo[269].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Enable/Cursor Enable Read.vi</Property>
				<Property Name="FileInfo[269].Type" Type="Int">3</Property>
				<Property Name="FileInfo[269].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[27].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[27].FileName" Type="Str">Active Light Read.vi</Property>
				<Property Name="FileInfo[27].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Active Light Read.vi</Property>
				<Property Name="FileInfo[27].Type" Type="Int">3</Property>
				<Property Name="FileInfo[27].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[270].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[270].FileName" Type="Str">Cursor Enable Write.vi</Property>
				<Property Name="FileInfo[270].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Enable/Cursor Enable Write.vi</Property>
				<Property Name="FileInfo[270].Type" Type="Int">3</Property>
				<Property Name="FileInfo[270].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[271].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[271].FileName" Type="Str">Cursor Visible Read.vi</Property>
				<Property Name="FileInfo[271].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Visible/Cursor Visible Read.vi</Property>
				<Property Name="FileInfo[271].Type" Type="Int">3</Property>
				<Property Name="FileInfo[271].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[272].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[272].FileName" Type="Str">Cursor Visible Write.vi</Property>
				<Property Name="FileInfo[272].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Visible/Cursor Visible Write.vi</Property>
				<Property Name="FileInfo[272].Type" Type="Int">3</Property>
				<Property Name="FileInfo[272].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[273].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[273].FileName" Type="Str">Cursor Lock Style Read.vi</Property>
				<Property Name="FileInfo[273].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Lock Style/Cursor Lock Style Read.vi</Property>
				<Property Name="FileInfo[273].Type" Type="Int">3</Property>
				<Property Name="FileInfo[273].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[274].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[274].FileName" Type="Str">Cursor Lock Style Write.vi</Property>
				<Property Name="FileInfo[274].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Lock Style/Cursor Lock Style Write.vi</Property>
				<Property Name="FileInfo[274].Type" Type="Int">3</Property>
				<Property Name="FileInfo[274].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[275].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[275].FileName" Type="Str">Cursor Plot ID Read.vi</Property>
				<Property Name="FileInfo[275].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plot ID/Cursor Plot ID Read.vi</Property>
				<Property Name="FileInfo[275].Type" Type="Int">3</Property>
				<Property Name="FileInfo[275].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[276].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[276].FileName" Type="Str">Cursor Plot ID Write.vi</Property>
				<Property Name="FileInfo[276].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plot ID/Cursor Plot ID Write.vi</Property>
				<Property Name="FileInfo[276].Type" Type="Int">3</Property>
				<Property Name="FileInfo[276].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[277].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[277].FileName" Type="Str">Cursor X Position Read.vi</Property>
				<Property Name="FileInfo[277].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor X Position/Cursor X Position Read.vi</Property>
				<Property Name="FileInfo[277].Type" Type="Int">3</Property>
				<Property Name="FileInfo[277].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[278].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[278].FileName" Type="Str">Cursor X Position Write.vi</Property>
				<Property Name="FileInfo[278].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor X Position/Cursor X Position Write.vi</Property>
				<Property Name="FileInfo[278].Type" Type="Int">3</Property>
				<Property Name="FileInfo[278].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[279].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[279].FileName" Type="Str">Cursor Y Position Read.vi</Property>
				<Property Name="FileInfo[279].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Y Position/Cursor Y Position Read.vi</Property>
				<Property Name="FileInfo[279].Type" Type="Int">3</Property>
				<Property Name="FileInfo[279].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[28].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[28].FileName" Type="Str">Active Light Write.vi</Property>
				<Property Name="FileInfo[28].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Active Light Write.vi</Property>
				<Property Name="FileInfo[28].Type" Type="Int">3</Property>
				<Property Name="FileInfo[28].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[280].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[280].FileName" Type="Str">Cursor Y Position Write.vi</Property>
				<Property Name="FileInfo[280].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Y Position/Cursor Y Position Write.vi</Property>
				<Property Name="FileInfo[280].Type" Type="Int">3</Property>
				<Property Name="FileInfo[280].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[281].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[281].FileName" Type="Str">Cursor Z Position Read.vi</Property>
				<Property Name="FileInfo[281].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Z Position/Cursor Z Position Read.vi</Property>
				<Property Name="FileInfo[281].Type" Type="Int">3</Property>
				<Property Name="FileInfo[281].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[282].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[282].FileName" Type="Str">Cursor Z Position Write.vi</Property>
				<Property Name="FileInfo[282].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Z Position/Cursor Z Position Write.vi</Property>
				<Property Name="FileInfo[282].Type" Type="Int">3</Property>
				<Property Name="FileInfo[282].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[283].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[283].FileName" Type="Str">Cursor Point Color Read.vi</Property>
				<Property Name="FileInfo[283].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Color/Cursor Point Color Read.vi</Property>
				<Property Name="FileInfo[283].Type" Type="Int">3</Property>
				<Property Name="FileInfo[283].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[284].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[284].FileName" Type="Str">Cursor Point Color Write.vi</Property>
				<Property Name="FileInfo[284].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Color/Cursor Point Color Write.vi</Property>
				<Property Name="FileInfo[284].Type" Type="Int">3</Property>
				<Property Name="FileInfo[284].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[285].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[285].FileName" Type="Str">Cursor Point Size Read.vi</Property>
				<Property Name="FileInfo[285].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Size/Cursor Point Size Read.vi</Property>
				<Property Name="FileInfo[285].Type" Type="Int">3</Property>
				<Property Name="FileInfo[285].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[286].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[286].FileName" Type="Str">Cursor Point Size Write.vi</Property>
				<Property Name="FileInfo[286].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Size/Cursor Point Size Write.vi</Property>
				<Property Name="FileInfo[286].Type" Type="Int">3</Property>
				<Property Name="FileInfo[286].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[287].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[287].FileName" Type="Str">Cursor Point Style Read.vi</Property>
				<Property Name="FileInfo[287].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Style/Cursor Point Style Read.vi</Property>
				<Property Name="FileInfo[287].Type" Type="Int">3</Property>
				<Property Name="FileInfo[287].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[288].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[288].FileName" Type="Str">Cursor Point Style Write.vi</Property>
				<Property Name="FileInfo[288].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Point Style/Cursor Point Style Write.vi</Property>
				<Property Name="FileInfo[288].Type" Type="Int">3</Property>
				<Property Name="FileInfo[288].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[289].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[289].FileName" Type="Str">Cursor Line Color Read.vi</Property>
				<Property Name="FileInfo[289].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Color/Cursor Line Color Read.vi</Property>
				<Property Name="FileInfo[289].Type" Type="Int">3</Property>
				<Property Name="FileInfo[289].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[29].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[29].FileName" Type="Str">Enable Light Read.vi</Property>
				<Property Name="FileInfo[29].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Enable Light Read.vi</Property>
				<Property Name="FileInfo[29].Type" Type="Int">3</Property>
				<Property Name="FileInfo[29].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[290].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[290].FileName" Type="Str">Cursor Line Color Write.vi</Property>
				<Property Name="FileInfo[290].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Color/Cursor Line Color Write.vi</Property>
				<Property Name="FileInfo[290].Type" Type="Int">3</Property>
				<Property Name="FileInfo[290].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[291].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[291].FileName" Type="Str">Cursor Line Width Read.vi</Property>
				<Property Name="FileInfo[291].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Width/Cursor Line Width Read.vi</Property>
				<Property Name="FileInfo[291].Type" Type="Int">3</Property>
				<Property Name="FileInfo[291].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[292].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[292].FileName" Type="Str">Cursor Line Width Write.vi</Property>
				<Property Name="FileInfo[292].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Width/Cursor Line Width Write.vi</Property>
				<Property Name="FileInfo[292].Type" Type="Int">3</Property>
				<Property Name="FileInfo[292].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[293].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[293].FileName" Type="Str">Cursor Line Style Read.vi</Property>
				<Property Name="FileInfo[293].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Style/Cursor Line Style Read.vi</Property>
				<Property Name="FileInfo[293].Type" Type="Int">3</Property>
				<Property Name="FileInfo[293].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[294].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[294].FileName" Type="Str">Cursor Line Style Write.vi</Property>
				<Property Name="FileInfo[294].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Line Style/Cursor Line Style Write.vi</Property>
				<Property Name="FileInfo[294].Type" Type="Int">3</Property>
				<Property Name="FileInfo[294].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[295].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[295].FileName" Type="Str">Cursor Plane Color Read.vi</Property>
				<Property Name="FileInfo[295].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane Color/Cursor Plane Color Read.vi</Property>
				<Property Name="FileInfo[295].Type" Type="Int">3</Property>
				<Property Name="FileInfo[295].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[296].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[296].FileName" Type="Str">Cursor Plane Color Write.vi</Property>
				<Property Name="FileInfo[296].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane Color/Cursor Plane Color Write.vi</Property>
				<Property Name="FileInfo[296].Type" Type="Int">3</Property>
				<Property Name="FileInfo[296].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[297].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[297].FileName" Type="Str">Cursor Plane Opacity Read.vi</Property>
				<Property Name="FileInfo[297].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane Opacity/Cursor Plane Opacity Read.vi</Property>
				<Property Name="FileInfo[297].Type" Type="Int">3</Property>
				<Property Name="FileInfo[297].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[298].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[298].FileName" Type="Str">Cursor Plane Opacity Write.vi</Property>
				<Property Name="FileInfo[298].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane Opacity/Cursor Plane Opacity Write.vi</Property>
				<Property Name="FileInfo[298].Type" Type="Int">3</Property>
				<Property Name="FileInfo[298].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[299].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[299].FileName" Type="Str">Cursor Plane XY Visible Read.vi</Property>
				<Property Name="FileInfo[299].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane XY Visible/Cursor Plane XY Visible Read.vi</Property>
				<Property Name="FileInfo[299].Type" Type="Int">3</Property>
				<Property Name="FileInfo[299].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[3].DirTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="FileInfo[3].FileName" Type="Str">AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[3].FileTag" Type="Ref">/My Computer/AeroQuadConfigurator.ini</Property>
				<Property Name="FileInfo[3].Type" Type="Int">3</Property>
				<Property Name="FileInfo[3].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[30].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[30].FileName" Type="Str">Enable Light Write.vi</Property>
				<Property Name="FileInfo[30].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Enable Light Write.vi</Property>
				<Property Name="FileInfo[30].Type" Type="Int">3</Property>
				<Property Name="FileInfo[30].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[300].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[300].FileName" Type="Str">Cursor Plane XY Visible Write.vi</Property>
				<Property Name="FileInfo[300].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane XY Visible/Cursor Plane XY Visible Write.vi</Property>
				<Property Name="FileInfo[300].Type" Type="Int">3</Property>
				<Property Name="FileInfo[300].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[301].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[301].FileName" Type="Str">Cursor Plane XZ Visible Read.vi</Property>
				<Property Name="FileInfo[301].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane XZ Visible/Cursor Plane XZ Visible Read.vi</Property>
				<Property Name="FileInfo[301].Type" Type="Int">3</Property>
				<Property Name="FileInfo[301].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[302].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[302].FileName" Type="Str">Cursor Plane XZ Visible Write.vi</Property>
				<Property Name="FileInfo[302].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane XZ Visible/Cursor Plane XZ Visible Write.vi</Property>
				<Property Name="FileInfo[302].Type" Type="Int">3</Property>
				<Property Name="FileInfo[302].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[303].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[303].FileName" Type="Str">Cursor Plane YZ Visible Read.vi</Property>
				<Property Name="FileInfo[303].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane YZ Visible/Cursor Plane YZ Visible Read.vi</Property>
				<Property Name="FileInfo[303].Type" Type="Int">3</Property>
				<Property Name="FileInfo[303].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[304].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[304].FileName" Type="Str">Cursor Plane YZ Visible Write.vi</Property>
				<Property Name="FileInfo[304].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Plane YZ Visible/Cursor Plane YZ Visible Write.vi</Property>
				<Property Name="FileInfo[304].Type" Type="Int">3</Property>
				<Property Name="FileInfo[304].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[305].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[305].FileName" Type="Str">Cursor Name Read.vi</Property>
				<Property Name="FileInfo[305].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Name/Cursor Name Read.vi</Property>
				<Property Name="FileInfo[305].Type" Type="Int">3</Property>
				<Property Name="FileInfo[305].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[306].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[306].FileName" Type="Str">Cursor Name Write.vi</Property>
				<Property Name="FileInfo[306].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Name/Cursor Name Write.vi</Property>
				<Property Name="FileInfo[306].Type" Type="Int">3</Property>
				<Property Name="FileInfo[306].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[307].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[307].FileName" Type="Str">Cursor Text Color Read.vi</Property>
				<Property Name="FileInfo[307].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Color/Cursor Text Color Read.vi</Property>
				<Property Name="FileInfo[307].Type" Type="Int">3</Property>
				<Property Name="FileInfo[307].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[308].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[308].FileName" Type="Str">Cursor Text Color Write.vi</Property>
				<Property Name="FileInfo[308].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Color/Cursor Text Color Write.vi</Property>
				<Property Name="FileInfo[308].Type" Type="Int">3</Property>
				<Property Name="FileInfo[308].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[309].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[309].FileName" Type="Str">Cursor Text Size Read.vi</Property>
				<Property Name="FileInfo[309].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Size/Cursor Text Size Read.vi</Property>
				<Property Name="FileInfo[309].Type" Type="Int">3</Property>
				<Property Name="FileInfo[309].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[31].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[31].FileName" Type="Str">Light Color Read.vi</Property>
				<Property Name="FileInfo[31].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Color Read.vi</Property>
				<Property Name="FileInfo[31].Type" Type="Int">3</Property>
				<Property Name="FileInfo[31].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[310].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[310].FileName" Type="Str">Cursor Text Size Write.vi</Property>
				<Property Name="FileInfo[310].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Size/Cursor Text Size Write.vi</Property>
				<Property Name="FileInfo[310].Type" Type="Int">3</Property>
				<Property Name="FileInfo[310].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[311].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[311].FileName" Type="Str">Cursor Show Position Read.vi</Property>
				<Property Name="FileInfo[311].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Show Position/Cursor Show Position Read.vi</Property>
				<Property Name="FileInfo[311].Type" Type="Int">3</Property>
				<Property Name="FileInfo[311].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[312].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[312].FileName" Type="Str">Cursor Show Position Write.vi</Property>
				<Property Name="FileInfo[312].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Show Position/Cursor Show Position Write.vi</Property>
				<Property Name="FileInfo[312].Type" Type="Int">3</Property>
				<Property Name="FileInfo[312].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[313].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[313].FileName" Type="Str">Cursor Show Name Read.vi</Property>
				<Property Name="FileInfo[313].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Show Name/Cursor Show Name Read.vi</Property>
				<Property Name="FileInfo[313].Type" Type="Int">3</Property>
				<Property Name="FileInfo[313].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[314].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[314].FileName" Type="Str">Cursor Show Name Write.vi</Property>
				<Property Name="FileInfo[314].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Show Name/Cursor Show Name Write.vi</Property>
				<Property Name="FileInfo[314].Type" Type="Int">3</Property>
				<Property Name="FileInfo[314].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[315].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[315].FileName" Type="Str">Cursor Text Background Color Read.vi</Property>
				<Property Name="FileInfo[315].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Background Color/Cursor Text Background Color Read.vi</Property>
				<Property Name="FileInfo[315].Type" Type="Int">3</Property>
				<Property Name="FileInfo[315].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[316].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[316].FileName" Type="Str">Cursor Text Background Color Write.vi</Property>
				<Property Name="FileInfo[316].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Background Color/Cursor Text Background Color Write.vi</Property>
				<Property Name="FileInfo[316].Type" Type="Int">3</Property>
				<Property Name="FileInfo[316].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[317].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[317].FileName" Type="Str">Cursor Text Background Opacity Read.vi</Property>
				<Property Name="FileInfo[317].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Background Opacity/Cursor Text Background Opacity Read.vi</Property>
				<Property Name="FileInfo[317].Type" Type="Int">3</Property>
				<Property Name="FileInfo[317].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[318].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[318].FileName" Type="Str">Cursor Text Background Opacity Write.vi</Property>
				<Property Name="FileInfo[318].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor Text Background Opacity/Cursor Text Background Opacity Write.vi</Property>
				<Property Name="FileInfo[318].Type" Type="Int">3</Property>
				<Property Name="FileInfo[318].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[319].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[319].FileName" Type="Str">Cursor List Read.vi</Property>
				<Property Name="FileInfo[319].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor List/Cursor List Read.vi</Property>
				<Property Name="FileInfo[319].Type" Type="Int">3</Property>
				<Property Name="FileInfo[319].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[32].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[32].FileName" Type="Str">Light Color Write.vi</Property>
				<Property Name="FileInfo[32].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Color Write.vi</Property>
				<Property Name="FileInfo[32].Type" Type="Int">3</Property>
				<Property Name="FileInfo[32].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[320].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[320].FileName" Type="Str">Cursor List Write.vi</Property>
				<Property Name="FileInfo[320].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Properties/Cursors/Cursor List/Cursor List Write.vi</Property>
				<Property Name="FileInfo[320].Type" Type="Int">3</Property>
				<Property Name="FileInfo[320].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[321].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[321].FileName" Type="Str">Image to Printer.vi</Property>
				<Property Name="FileInfo[321].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Methods/Image to Printer.vi</Property>
				<Property Name="FileInfo[321].Type" Type="Int">3</Property>
				<Property Name="FileInfo[321].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[322].DirTag" Type="Str">{5509E495-2C08-4EA9-9EC6-3EF286E5F833}</Property>
				<Property Name="FileInfo[322].FileName" Type="Str">Image to File.vi</Property>
				<Property Name="FileInfo[322].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter.xctl/Methods/Image to File.vi</Property>
				<Property Name="FileInfo[322].Type" Type="Int">3</Property>
				<Property Name="FileInfo[322].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[323].DirTag" Type="Str">{003DAD04-8A37-45BD-840C-25A67C790C1B}</Property>
				<Property Name="FileInfo[323].FileName" Type="Str">Calculate Vertex Array.vi</Property>
				<Property Name="FileInfo[323].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter Datatype.lvclass/Protected/Create Plot Data/Calculate Vertex Array.vi</Property>
				<Property Name="FileInfo[323].Type" Type="Int">3</Property>
				<Property Name="FileInfo[323].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[324].DirTag" Type="Str">{003DAD04-8A37-45BD-840C-25A67C790C1B}</Property>
				<Property Name="FileInfo[324].FileName" Type="Str">Calculate Color Index Array.vi</Property>
				<Property Name="FileInfo[324].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter Datatype.lvclass/Protected/Create Plot Data/Calculate Color Index Array.vi</Property>
				<Property Name="FileInfo[324].Type" Type="Int">3</Property>
				<Property Name="FileInfo[324].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[325].DirTag" Type="Str">{003DAD04-8A37-45BD-840C-25A67C790C1B}</Property>
				<Property Name="FileInfo[325].FileName" Type="Str">Get Plot ID.vi</Property>
				<Property Name="FileInfo[325].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Scatter Datatype.lvclass/Protected/Create Plot Data/Get Plot ID.vi</Property>
				<Property Name="FileInfo[325].Type" Type="Int">3</Property>
				<Property Name="FileInfo[325].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[326].DirTag" Type="Str">{4A68D4FE-0F2D-4EEE-9E57-C6E5EEE95FFF}</Property>
				<Property Name="FileInfo[326].FileName" Type="Str">Create Clip Plane.vi</Property>
				<Property Name="FileInfo[326].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/NI_3D Picture Control.lvlib/Helpers/Create Clip Plane.vi</Property>
				<Property Name="FileInfo[326].Type" Type="Int">3</Property>
				<Property Name="FileInfo[326].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[33].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[33].FileName" Type="Str">Light Latitude Read.vi</Property>
				<Property Name="FileInfo[33].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Latitude Read.vi</Property>
				<Property Name="FileInfo[33].Type" Type="Int">3</Property>
				<Property Name="FileInfo[33].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[34].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[34].FileName" Type="Str">Light Latitude Write.vi</Property>
				<Property Name="FileInfo[34].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Latitude Write.vi</Property>
				<Property Name="FileInfo[34].Type" Type="Int">3</Property>
				<Property Name="FileInfo[34].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[35].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[35].FileName" Type="Str">Light Longitude Read.vi</Property>
				<Property Name="FileInfo[35].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Longitude Read.vi</Property>
				<Property Name="FileInfo[35].Type" Type="Int">3</Property>
				<Property Name="FileInfo[35].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[36].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[36].FileName" Type="Str">Light Longitude Write.vi</Property>
				<Property Name="FileInfo[36].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Longitude Write.vi</Property>
				<Property Name="FileInfo[36].Type" Type="Int">3</Property>
				<Property Name="FileInfo[36].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[37].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[37].FileName" Type="Str">Light Distance Read.vi</Property>
				<Property Name="FileInfo[37].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Distance Read.vi</Property>
				<Property Name="FileInfo[37].Type" Type="Int">3</Property>
				<Property Name="FileInfo[37].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[38].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[38].FileName" Type="Str">Light Distance Write.vi</Property>
				<Property Name="FileInfo[38].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Distance Write.vi</Property>
				<Property Name="FileInfo[38].Type" Type="Int">3</Property>
				<Property Name="FileInfo[38].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[39].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[39].FileName" Type="Str">Light Attenuation Read.vi</Property>
				<Property Name="FileInfo[39].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Attenuation Read.vi</Property>
				<Property Name="FileInfo[39].Type" Type="Int">3</Property>
				<Property Name="FileInfo[39].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[4].DirTag" Type="Str">{41D76EAD-01B6-44A8-A571-C2D7BE7C0911}</Property>
				<Property Name="FileInfo[4].FileName" Type="Str">AeroQuadConfigurator.vi</Property>
				<Property Name="FileInfo[4].FileTag" Type="Ref">/My Computer/AeroQuadConfigurator.vi</Property>
				<Property Name="FileInfo[4].Type" Type="Int">3</Property>
				<Property Name="FileInfo[4].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[40].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[40].FileName" Type="Str">Light Attenuation Write.vi</Property>
				<Property Name="FileInfo[40].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Light/Light Attenuation Write.vi</Property>
				<Property Name="FileInfo[40].Type" Type="Int">3</Property>
				<Property Name="FileInfo[40].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[41].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[41].FileName" Type="Str">Active Axis Read.vi</Property>
				<Property Name="FileInfo[41].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Active Axis Read.vi</Property>
				<Property Name="FileInfo[41].Type" Type="Int">3</Property>
				<Property Name="FileInfo[41].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[42].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[42].FileName" Type="Str">Active Axis Write.vi</Property>
				<Property Name="FileInfo[42].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Active Axis Write.vi</Property>
				<Property Name="FileInfo[42].Type" Type="Int">3</Property>
				<Property Name="FileInfo[42].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[43].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[43].FileName" Type="Str">Name Label Color Read.vi</Property>
				<Property Name="FileInfo[43].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Color Read.vi</Property>
				<Property Name="FileInfo[43].Type" Type="Int">3</Property>
				<Property Name="FileInfo[43].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[44].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[44].FileName" Type="Str">Name Label Color Write.vi</Property>
				<Property Name="FileInfo[44].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Color Write.vi</Property>
				<Property Name="FileInfo[44].Type" Type="Int">3</Property>
				<Property Name="FileInfo[44].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[45].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[45].FileName" Type="Str">Name Label Size Read.vi</Property>
				<Property Name="FileInfo[45].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Size Read.vi</Property>
				<Property Name="FileInfo[45].Type" Type="Int">3</Property>
				<Property Name="FileInfo[45].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[46].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[46].FileName" Type="Str">Name Label Size Write.vi</Property>
				<Property Name="FileInfo[46].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Size Write.vi</Property>
				<Property Name="FileInfo[46].Type" Type="Int">3</Property>
				<Property Name="FileInfo[46].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[47].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[47].FileName" Type="Str">Name Label Text Read.vi</Property>
				<Property Name="FileInfo[47].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Text Read.vi</Property>
				<Property Name="FileInfo[47].Type" Type="Int">3</Property>
				<Property Name="FileInfo[47].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[48].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[48].FileName" Type="Str">Name Label Text Write.vi</Property>
				<Property Name="FileInfo[48].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Text Write.vi</Property>
				<Property Name="FileInfo[48].Type" Type="Int">3</Property>
				<Property Name="FileInfo[48].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[49].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[49].FileName" Type="Str">Name Label Normal Visible Read.vi</Property>
				<Property Name="FileInfo[49].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Normal Visible Read.vi</Property>
				<Property Name="FileInfo[49].Type" Type="Int">3</Property>
				<Property Name="FileInfo[49].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[5].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[5].FileName" Type="Str">Fast Draw Read.vi</Property>
				<Property Name="FileInfo[5].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Fast Draw Read.vi</Property>
				<Property Name="FileInfo[5].Type" Type="Int">3</Property>
				<Property Name="FileInfo[5].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[50].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[50].FileName" Type="Str">Name Label Normal Visible Write.vi</Property>
				<Property Name="FileInfo[50].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Normal Visible Write.vi</Property>
				<Property Name="FileInfo[50].Type" Type="Int">3</Property>
				<Property Name="FileInfo[50].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[51].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[51].FileName" Type="Str">Name Label Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[51].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[51].Type" Type="Int">3</Property>
				<Property Name="FileInfo[51].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[52].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[52].FileName" Type="Str">Name Label Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[52].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Name Label Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[52].Type" Type="Int">3</Property>
				<Property Name="FileInfo[52].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[53].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[53].FileName" Type="Str">Marker Color Read.vi</Property>
				<Property Name="FileInfo[53].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Color Read.vi</Property>
				<Property Name="FileInfo[53].Type" Type="Int">3</Property>
				<Property Name="FileInfo[53].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[54].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[54].FileName" Type="Str">Marker Color Write.vi</Property>
				<Property Name="FileInfo[54].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Color Write.vi</Property>
				<Property Name="FileInfo[54].Type" Type="Int">3</Property>
				<Property Name="FileInfo[54].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[55].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[55].FileName" Type="Str">Marker Size Read.vi</Property>
				<Property Name="FileInfo[55].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Size Read.vi</Property>
				<Property Name="FileInfo[55].Type" Type="Int">3</Property>
				<Property Name="FileInfo[55].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[56].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[56].FileName" Type="Str">Marker Size Write.vi</Property>
				<Property Name="FileInfo[56].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Size Write.vi</Property>
				<Property Name="FileInfo[56].Type" Type="Int">3</Property>
				<Property Name="FileInfo[56].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[57].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[57].FileName" Type="Str">Marker Normal Visible Read.vi</Property>
				<Property Name="FileInfo[57].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Normal Visible Read.vi</Property>
				<Property Name="FileInfo[57].Type" Type="Int">3</Property>
				<Property Name="FileInfo[57].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[58].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[58].FileName" Type="Str">Marker Normal Visible Write.vi</Property>
				<Property Name="FileInfo[58].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Normal Visible Write.vi</Property>
				<Property Name="FileInfo[58].Type" Type="Int">3</Property>
				<Property Name="FileInfo[58].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[59].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[59].FileName" Type="Str">Marker Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[59].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Opposite Visible Read.vi</Property>
				<Property Name="FileInfo[59].Type" Type="Int">3</Property>
				<Property Name="FileInfo[59].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[6].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[6].FileName" Type="Str">Fast Draw Write.vi</Property>
				<Property Name="FileInfo[6].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Fast Draw Write.vi</Property>
				<Property Name="FileInfo[6].Type" Type="Int">3</Property>
				<Property Name="FileInfo[6].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[60].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[60].FileName" Type="Str">Marker Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[60].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Marker Opposite Visible Write.vi</Property>
				<Property Name="FileInfo[60].Type" Type="Int">3</Property>
				<Property Name="FileInfo[60].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[61].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[61].FileName" Type="Str">Range Maximum Read.vi</Property>
				<Property Name="FileInfo[61].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Maximum Read.vi</Property>
				<Property Name="FileInfo[61].Type" Type="Int">3</Property>
				<Property Name="FileInfo[61].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[62].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[62].FileName" Type="Str">Range Maximum Write.vi</Property>
				<Property Name="FileInfo[62].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Maximum Write.vi</Property>
				<Property Name="FileInfo[62].Type" Type="Int">3</Property>
				<Property Name="FileInfo[62].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[63].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[63].FileName" Type="Str">Range Minimum Read.vi</Property>
				<Property Name="FileInfo[63].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Minimum Read.vi</Property>
				<Property Name="FileInfo[63].Type" Type="Int">3</Property>
				<Property Name="FileInfo[63].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[64].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[64].FileName" Type="Str">Range Minimum Write.vi</Property>
				<Property Name="FileInfo[64].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Minimum Write.vi</Property>
				<Property Name="FileInfo[64].Type" Type="Int">3</Property>
				<Property Name="FileInfo[64].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[65].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[65].FileName" Type="Str">Range Inverted Read.vi</Property>
				<Property Name="FileInfo[65].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Inverted Read.vi</Property>
				<Property Name="FileInfo[65].Type" Type="Int">3</Property>
				<Property Name="FileInfo[65].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[66].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[66].FileName" Type="Str">Range Inverted Write.vi</Property>
				<Property Name="FileInfo[66].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Inverted Write.vi</Property>
				<Property Name="FileInfo[66].Type" Type="Int">3</Property>
				<Property Name="FileInfo[66].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[67].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[67].FileName" Type="Str">Range Logarithm Read.vi</Property>
				<Property Name="FileInfo[67].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Logarithm Read.vi</Property>
				<Property Name="FileInfo[67].Type" Type="Int">3</Property>
				<Property Name="FileInfo[67].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[68].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[68].FileName" Type="Str">Range Logarithm Write.vi</Property>
				<Property Name="FileInfo[68].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Range Logarithm Write.vi</Property>
				<Property Name="FileInfo[68].Type" Type="Int">3</Property>
				<Property Name="FileInfo[68].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[69].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[69].FileName" Type="Str">Grid Major Color Read.vi</Property>
				<Property Name="FileInfo[69].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Major Color Read.vi</Property>
				<Property Name="FileInfo[69].Type" Type="Int">3</Property>
				<Property Name="FileInfo[69].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[7].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[7].FileName" Type="Str">Clip Data Read.vi</Property>
				<Property Name="FileInfo[7].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Clip Data Read.vi</Property>
				<Property Name="FileInfo[7].Type" Type="Int">3</Property>
				<Property Name="FileInfo[7].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[70].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[70].FileName" Type="Str">Grid Major Color Write.vi</Property>
				<Property Name="FileInfo[70].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Major Color Write.vi</Property>
				<Property Name="FileInfo[70].Type" Type="Int">3</Property>
				<Property Name="FileInfo[70].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[71].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[71].FileName" Type="Str">Grid Major Visible Read.vi</Property>
				<Property Name="FileInfo[71].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Major Visible Read.vi</Property>
				<Property Name="FileInfo[71].Type" Type="Int">3</Property>
				<Property Name="FileInfo[71].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[72].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[72].FileName" Type="Str">Grid Major Visible Write.vi</Property>
				<Property Name="FileInfo[72].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Major Visible Write.vi</Property>
				<Property Name="FileInfo[72].Type" Type="Int">3</Property>
				<Property Name="FileInfo[72].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[73].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[73].FileName" Type="Str">Grid Minor Color Read.vi</Property>
				<Property Name="FileInfo[73].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Minor Color Read.vi</Property>
				<Property Name="FileInfo[73].Type" Type="Int">3</Property>
				<Property Name="FileInfo[73].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[74].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[74].FileName" Type="Str">Grid Minor Color Write.vi</Property>
				<Property Name="FileInfo[74].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Minor Color Write.vi</Property>
				<Property Name="FileInfo[74].Type" Type="Int">3</Property>
				<Property Name="FileInfo[74].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[75].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[75].FileName" Type="Str">Grid Minor Visible Read.vi</Property>
				<Property Name="FileInfo[75].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Minor Visible Read.vi</Property>
				<Property Name="FileInfo[75].Type" Type="Int">3</Property>
				<Property Name="FileInfo[75].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[76].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[76].FileName" Type="Str">Grid Minor Visible Write.vi</Property>
				<Property Name="FileInfo[76].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Grid Minor Visible Write.vi</Property>
				<Property Name="FileInfo[76].Type" Type="Int">3</Property>
				<Property Name="FileInfo[76].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[77].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[77].FileName" Type="Str">Tick Major Count Read.vi</Property>
				<Property Name="FileInfo[77].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Count Read.vi</Property>
				<Property Name="FileInfo[77].Type" Type="Int">3</Property>
				<Property Name="FileInfo[77].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[78].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[78].FileName" Type="Str">Tick Major Count Write.vi</Property>
				<Property Name="FileInfo[78].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Count Write.vi</Property>
				<Property Name="FileInfo[78].Type" Type="Int">3</Property>
				<Property Name="FileInfo[78].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[79].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[79].FileName" Type="Str">Tick Major Color Read.vi</Property>
				<Property Name="FileInfo[79].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Color Read.vi</Property>
				<Property Name="FileInfo[79].Type" Type="Int">3</Property>
				<Property Name="FileInfo[79].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[8].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[8].FileName" Type="Str">Clip Data Write.vi</Property>
				<Property Name="FileInfo[8].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Clip Data Write.vi</Property>
				<Property Name="FileInfo[8].Type" Type="Int">3</Property>
				<Property Name="FileInfo[8].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[80].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[80].FileName" Type="Str">Tick Major Color Write.vi</Property>
				<Property Name="FileInfo[80].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Color Write.vi</Property>
				<Property Name="FileInfo[80].Type" Type="Int">3</Property>
				<Property Name="FileInfo[80].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[81].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[81].FileName" Type="Str">Tick Major Visible Read.vi</Property>
				<Property Name="FileInfo[81].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Visible Read.vi</Property>
				<Property Name="FileInfo[81].Type" Type="Int">3</Property>
				<Property Name="FileInfo[81].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[82].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[82].FileName" Type="Str">Tick Major Visible Write.vi</Property>
				<Property Name="FileInfo[82].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Major Visible Write.vi</Property>
				<Property Name="FileInfo[82].Type" Type="Int">3</Property>
				<Property Name="FileInfo[82].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[83].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[83].FileName" Type="Str">Tick Minor Count Read.vi</Property>
				<Property Name="FileInfo[83].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Count Read.vi</Property>
				<Property Name="FileInfo[83].Type" Type="Int">3</Property>
				<Property Name="FileInfo[83].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[84].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[84].FileName" Type="Str">Tick Minor Count Write.vi</Property>
				<Property Name="FileInfo[84].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Count Write.vi</Property>
				<Property Name="FileInfo[84].Type" Type="Int">3</Property>
				<Property Name="FileInfo[84].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[85].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[85].FileName" Type="Str">Tick Minor Color Read.vi</Property>
				<Property Name="FileInfo[85].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Color Read.vi</Property>
				<Property Name="FileInfo[85].Type" Type="Int">3</Property>
				<Property Name="FileInfo[85].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[86].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[86].FileName" Type="Str">Tick Minor Color Write.vi</Property>
				<Property Name="FileInfo[86].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Color Write.vi</Property>
				<Property Name="FileInfo[86].Type" Type="Int">3</Property>
				<Property Name="FileInfo[86].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[87].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[87].FileName" Type="Str">Tick Minor Visible Read.vi</Property>
				<Property Name="FileInfo[87].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Visible Read.vi</Property>
				<Property Name="FileInfo[87].Type" Type="Int">3</Property>
				<Property Name="FileInfo[87].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[88].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[88].FileName" Type="Str">Tick Minor Visible Write.vi</Property>
				<Property Name="FileInfo[88].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Axis/Tick Minor Visible Write.vi</Property>
				<Property Name="FileInfo[88].Type" Type="Int">3</Property>
				<Property Name="FileInfo[88].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[89].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[89].FileName" Type="Str">Active Value Pair Read.vi</Property>
				<Property Name="FileInfo[89].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Active Value Pair Read.vi</Property>
				<Property Name="FileInfo[89].Type" Type="Int">3</Property>
				<Property Name="FileInfo[89].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[9].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[9].FileName" Type="Str">Projection Mode Read.vi</Property>
				<Property Name="FileInfo[9].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Graph/Projection Mode Read.vi</Property>
				<Property Name="FileInfo[9].Type" Type="Int">3</Property>
				<Property Name="FileInfo[9].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[90].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[90].FileName" Type="Str">Active Value Pair Write.vi</Property>
				<Property Name="FileInfo[90].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Active Value Pair Write.vi</Property>
				<Property Name="FileInfo[90].Type" Type="Int">3</Property>
				<Property Name="FileInfo[90].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[91].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[91].FileName" Type="Str">Value Pair Label Type Read.vi</Property>
				<Property Name="FileInfo[91].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Label Type Read.vi</Property>
				<Property Name="FileInfo[91].Type" Type="Int">3</Property>
				<Property Name="FileInfo[91].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[92].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[92].FileName" Type="Str">Value Pair Label Type Write.vi</Property>
				<Property Name="FileInfo[92].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Label Type Write.vi</Property>
				<Property Name="FileInfo[92].Type" Type="Int">3</Property>
				<Property Name="FileInfo[92].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[93].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[93].FileName" Type="Str">Value Pair Tick Visible Read.vi</Property>
				<Property Name="FileInfo[93].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Tick Visible Read.vi</Property>
				<Property Name="FileInfo[93].Type" Type="Int">3</Property>
				<Property Name="FileInfo[93].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[94].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[94].FileName" Type="Str">Value Pair Tick Visible Write.vi</Property>
				<Property Name="FileInfo[94].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Tick Visible Write.vi</Property>
				<Property Name="FileInfo[94].Type" Type="Int">3</Property>
				<Property Name="FileInfo[94].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[95].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[95].FileName" Type="Str">Value Pair Grid Visible Read.vi</Property>
				<Property Name="FileInfo[95].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Grid Visible Read.vi</Property>
				<Property Name="FileInfo[95].Type" Type="Int">3</Property>
				<Property Name="FileInfo[95].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[96].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[96].FileName" Type="Str">Value Pair Grid Visible Write.vi</Property>
				<Property Name="FileInfo[96].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Grid Visible Write.vi</Property>
				<Property Name="FileInfo[96].Type" Type="Int">3</Property>
				<Property Name="FileInfo[96].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[97].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[97].FileName" Type="Str">Value Pair Array Read.vi</Property>
				<Property Name="FileInfo[97].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Array Read.vi</Property>
				<Property Name="FileInfo[97].Type" Type="Int">3</Property>
				<Property Name="FileInfo[97].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[98].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[98].FileName" Type="Str">Value Pair Array Write.vi</Property>
				<Property Name="FileInfo[98].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Value Pairs/Value Pair Array Write.vi</Property>
				<Property Name="FileInfo[98].Type" Type="Int">3</Property>
				<Property Name="FileInfo[98].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="FileInfo[99].DirTag" Type="Str">{773F72A5-40B2-4EF5-BD68-E7DCF0300269}</Property>
				<Property Name="FileInfo[99].FileName" Type="Str">Active Cursor Read.vi</Property>
				<Property Name="FileInfo[99].FileTag" Type="Ref">/My Computer/Dependencies/vi.lib/3D Plot.lvclass/Public/Property/Cursors/Active Cursor Read.vi</Property>
				<Property Name="FileInfo[99].Type" Type="Int">3</Property>
				<Property Name="FileInfo[99].TypeID" Type="Ref">/My Computer/Build Specifications/Executable</Property>
				<Property Name="InstSpecVersion" Type="Str">9018011</Property>
				<Property Name="LicenseFile" Type="Ref"></Property>
				<Property Name="OSCheck" Type="Int">0</Property>
				<Property Name="OSCheck_Vista" Type="Bool">false</Property>
				<Property Name="ProductName" Type="Str">AeroQuad Configurator</Property>
				<Property Name="ProductVersion" Type="Str">2.6.7</Property>
				<Property Name="ReadmeFile" Type="Ref"></Property>
				<Property Name="ShortcutInfo.Count" Type="Int">1</Property>
				<Property Name="ShortcutInfo[0].DirTag" Type="Str">{B9E310F1-839C-48B7-8CAE-33000780C26E}</Property>
				<Property Name="ShortcutInfo[0].FileTag" Type="Str">{66C02D92-29AD-43F8-A6C3-B835B0C6FAEA}</Property>
				<Property Name="ShortcutInfo[0].FileTagDir" Type="Str">{911FD540-AAC1-4ACE-952C-14E573DC1E9B}</Property>
				<Property Name="ShortcutInfo[0].Name" Type="Str">AeroQuad Configurator</Property>
				<Property Name="ShortcutInfo[0].SubDir" Type="Str">AeroQuad Configurator</Property>
				<Property Name="UpgradeCode" Type="Str">{70753F99-9091-4EE6-9377-3EF9476BEA88}</Property>
				<Property Name="WindowMessage" Type="Str">Thanks for using the AeroQuad Configurator.  The Configurator is used to setup your AeroQuad.  If you have any questions or suggestions for improvement for the Configurator, please post them at http://AeroQuad.com/forum.php</Property>
				<Property Name="WindowTitle" Type="Str">AeroQuad Configurator</Property>
			</Item>
		</Item>
	</Item>
</Project>
