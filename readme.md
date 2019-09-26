# EmonLibCM

The EmonLibCM library is maintained by @Robert.Wall and mirrored here for users of github.

See forum release: [EmonLibCM - Version 2.02](https://community.openenergymonitor.org/t/emonlibcm-version-2-02/9241)

The following release notes are copied from the forum thread. The installation section is modified to reflect git command line installation.

---

**EmonLibCM Release notes**<br>
EmonLibCM is a Continuous Monitoring alternative to EmonLib. Whereas emonLib repeats, every 5 or 10 s, a sequence of voltage and current measurements in each of the input channels for a short period, normally 200 ms, and then reports the measurements back to the sketch for onwards transmission to (for example) emonCMS; emonLibCM continuously measures in the background the voltage and all the current input channels in turn, calculates a true average quantity for each and then informs the sketch that the measurements are available and should be read and processed.

Temperature measurement with up to 6 DS18B20 sensors, and pulse counting, is included in the library. Neither must be added separately in the sketch.

The CM library will always give an accurate measurement of the average over the reporting period of the voltage, and for each of up to 5 channels the current, real and apparent power and power factor. A cumulative total of Watt-hours for each channel is also available. It is suitable for single phase or split phase operation at 50 Hz or 60 Hz. It will give more accurate values than the 'discrete sample' sketch where rapidly switched loads are in use, for example a burst mode energy diverter or an induction hob. 

The inputs can be calibrated for any realistic voltage and current, the default calibration is for an emonTx with a UK a.c. adapter and 100 A : 50 mA current transformers.

The library is distributed by @Robert.Wall as a compressed Zip file in the forum thread above. This contains the library files themselves (emonLibCM.cpp & emonLibCM.h), two directories with example sketches and a PDF format User Documentation file that contains notes on using the library, a full description of each method, instructions both for setting the initial configuration and for calibration, and brief notes explaining the example sketches. EmonLibCM also depends on several other libraries, and these are listed.

---

**INSTALLING THE LIBRARY (Forum thread Zip file)**<br>
The directory emonLibCM together with its contents should be extracted from the zip file and copied into the "libraries" directory, alongside (in the same level of the hierarchy as) the emonLib directory.

If you wish to use the example sketches, these (in their respective directories) should be moved or copied into your Sketchbook.

The User Documentation PDF file can be moved or copied to a convenient location of your choosing.

<a class="attachment" href="https://community.openenergymonitor.org/uploads/short-url/gofCd2DlmCWduHrhNRDPpBtxQeQ.zip">emonLibCM.zip</a> (Version 2: 130.2 KB)


Robert Wall will post updates on the forum thread first.


**INSTALLING THE LIBRARY (GitHub)**<br>
Navigate to your Arduino libraries directory and clone this repository:

    git clone https://github.com/openenergymonitor/EmonLibCM.git
    
Reload Arduino to start using the library.

---

**CHANGES**<br>
Version 2.0 corrected some errors generated when converting from the original sketch, and incorporates improved handling of phase/timing compensation and improved removal of the d.c. bias. There are no other major changes from the version that has been tested by @TrystanLea since early 2017.
Changes for V2.01: Errors in phase error correction.
Changes for V2.02: Temperature measurement: Added “BAD_TEMPERATURE” return value when reporting period < 0.2 s, getLogicalChannel( ), ReCalibrate_VChannel( ), ReCalibrate_IChannel( ) added, setPulsePin( ) interrupt no. was obligatory, pulse & temperatures were set/enabled only at startup, setTemperatureDataPin was ineffective, preloaded sensor addresses were not handled properly.

**USING THE LIBRARY**<br>
Two example sketches are provided as part of the distribution:

**EmonTxV34CM_min** is the absolute minimum sketch required to exercise the library and produce meaningful values.

**EmonTxV34CM_max** gives an example of every API call available. However, as distributed, it actually changes nothing as everything is again given the default value. If you need to change one of the defaults, then only the API call that sets that value is needed, and you can copy and add that call to the "minimum’ demo sketch.
The EmonLibCM library is not a direct replacement for the ‘discrete sample’ library emonLib. Significant changes will be need to be made if emonLibCM is to replace emonLib in any particular sketch.
The example sketches are intended only as a demonstration of the library. They do not (for example) take any account of the DIP switch settings of the emonTx V3.4. Great care must be taken if any significant additional load is to be put on the processor.

&nbsp;

**Acknowledgements.**<br>
Jörg Becker (@joergbecker32) for his background work on interrupts and the ADC.<br>
Robin Emley (@calypso_rae) for his energy diverter software, from which the major part of the library was derived by @TrystanLea<br> 
@ursi (Andries) and @mafheldt (Mike Afheldt) for suggestions made at https://community.openenergymonitor.org/t/emonlib-inaccurate-power-factor/3790 and https://community.openenergymonitor.org/t/rms-calculations-in-emonlib-and-learn-documentation/3749/3

&nbsp;

<table>
<th>MD5 Hashes:</th><th>&nbsp;</th>
<tr><td>EmonLibCM.cpp</td><td>9dd796794a90eab858a8dea60fdc2389</td></tr>
<tr><td>EmonLibCM.h</td><td> db208228f62bc5f804569e70efbcee63</td></tr>
<tr><td>emonLibCM User Doc.pdf</td><td>b7ed8b6f7004d0c9de38089c20695026</td></tr>
</table>

Please see [https://community.openenergymonitor.org/t/emonlibcm-version-2-support/9242?u=robert.wall](https://community.openenergymonitor.org/t/emonlibcm-version-2-support/9242?u=robert.wall) to comment or request support.
