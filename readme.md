# EmonLibCM

The EmonLibCM library is maintained by @Robert.Wall and mirrored here for users of github.

See forum release: [EmonLibCM - Version 2](https://community.openenergymonitor.org/t/emonlibcm-version-2/9241)

The following release notes are copied from the forum thread. The installation section is modified to reflect git command line installation.

---

EmonLibCM is a Continuous Monitoring alternative to EmonLib. Whereas emonLib repeats, every 5 or 10 s, a sequence of voltage and current measurements in each of the input channels for a short period, normally 200 ms, and then reports the measurements back to the sketch for onwards transmission to (for example) emonCMS; emonLibCM continuously measures in the background the voltage and all the current input channels in turn, calculates a true average quantity for each and then informs the sketch that the measurements are available and should be read and processed.

Temperature measurement with up to 6 DS18B20 sensors, and pulse counting, is included in the library. Neither must be added separately in the sketch.

The CM library will always give an accurate measurement of the average over the reporting period of the voltage, and for each of up to 5 channels the current, real and apparent power and power factor. A cumulative total of Watt-hours for each channel is also available. It is suitable for single phase or split phase operation at 50 Hz or 60 Hz. It will give more accurate values than the 'discrete sample' sketch where rapidly switched loads are in use, for example a burst mode energy diverter or an induction hob. 

The inputs can be calibrated for any realistic voltage and current, the default calibration is for an emonTx with a UK a.c. adapter and 100 A : 50 mA current transformers.

The library is distributed by @Robert.Wall as a compressed Zip file in the forum thread above. This contains the library files themselves (emonLibCM.cpp & emonLibCM.h), two directories with example sketches and a PDF format User Documentation file that contains notes on using the library, a full description of each method, instructions both for setting the initial configuration and for calibration, and brief notes explaining the example sketches. EmonLibCM also depends on several other libraries, and these are listed.

**INSTALLING THE LIBRARY (GitHub)**

Navigate to your Arduino libraries directory and clone this repository:

    git clone https://github.com/openenergymonitor/EmonLibCM.git
    
Reload Arduino to start using the library.

The User Documentation is available as a markdown document here:

[emonLibCM User Documentation](emonLibCM_User_Doc.md)

**INSTALLING THE LIBRARY (Forum thread Zip file)**

The directory emonLibCM together with its contents should be extracted from the zip file and copied into the "libraries" directory, alongside (in the same level of the hierarchy as) the emonLib directory.

If you wish to use the example sketches, these (in their respective directories) should be moved or copied into your Sketchbook.

The User Documentation PDF file can be moved or copied to a convenient location of your choosing.

<a class="attachment" href="https://community.openenergymonitor.org/uploads/default/original/2X/1/13d1b24912cbd3ce1f6f615c919904981a511688.zip">emonLibCM.zip</a> (Version 2: 125.5 KB)


Robert Wall will post updates on the forum thread first.

**CHANGES**
This version corrects some errors generated when converting from the original sketch, and incorporates improved handling of phase/timing compensation and improved removal of the d.c. bias. There are no other major changes from the version that has been tested by @TrystanLea since early 2017.

**USING THE LIBRARY**
Two example sketches are provided as part of the distribution:

**EmonTxV34CM_min** is the absolute minimum sketch required to exercise the library and produce meaningful values.

**EmonTxV34CM_max** gives an example of every API call available. However, as distributed, it actually changes nothing as everything is again given the default value. If you need to change one of the defaults, then only the API call that sets that value is needed, and you can copy and add that call to the "minimum' demo sketch.
The EmonLibCM library is not a direct replacement for the 'discrete sample' library emonLib. Significant changes will be need to be made if emonLibCM is to replace emonLib in any particular sketch.
The example sketches are intended only as a demonstration of the library. They do not (for example) take any account of the DIP switch settings of the emonTx V3.4. Great care must be taken if any significant additional load is to be put on the processor.

&nbsp;

**Acknowledgements.**
Jörg Becker (@joergbecker32) for his background work on interrupts and the ADC.
Robin Emley (@calypso_rae) for his energy diverter software, from which the major part of the library was derived by @TrystanLea 
@ursi (Andries) and @mafheldt (Mike Afheldt) for suggestions made at https://community.openenergymonitor.org/t/emonlib-inaccurate-power-factor/3790 and https://community.openenergymonitor.org/t/rms-calculations-in-emonlib-and-learn-documentation/3749/3

&nbsp;

<table>
<th>MD5 Hashes:</th><th>&nbsp;</th>
<tr><td>EmonLibCM.cpp</td><td>220c1a58553543dd1ed92203eceab77a</td></tr>
<tr><td>EmonLibCM.h</td><td> adcdfc3f940812a2d7cd3d1f96781df0</td></tr>
<tr><td>&nbsp;</td></tr>
<tr><td>emonLibCM User Doc.pdf</td><td>9e1d72a8450ff76272d1f44851a65013</td></tr>
</table>

Please see [https://community.openenergymonitor.org/t/emonlibcm-version-2-support/9242?u=robert.wall](https://community.openenergymonitor.org/t/emonlibcm-version-2-support/9242?u=robert.wall) to comment or request support.
