#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# Generated: Mon May 21 16:09:20 2018
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import uhd
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.wxgui import forms
from gnuradio.wxgui import scopesink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import EnergyBeamforming
import math
import threading
import time
import wx


class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.bf_weight = bf_weight = 1
        self.variable_static_text_0 = variable_static_text_0 = bf_weight
        self.tx_gain = tx_gain = 75
        self.tx_freq = tx_freq = 892e6
        self.samp_rate = samp_rate = 200000
        self.rx_freq = rx_freq = 2425.7275e6
        self.fsk_deviation_hz = fsk_deviation_hz = 38e3

        ##################################################
        # Blocks
        ##################################################
        self.EnergyBeamforming_randphpert4_f_0 = EnergyBeamforming.randphpert4_f(10, 1)
        _tx_gain_sizer = wx.BoxSizer(wx.VERTICAL)
        self._tx_gain_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_tx_gain_sizer,
        	value=self.tx_gain,
        	callback=self.set_tx_gain,
        	label="tx_gain",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._tx_gain_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_tx_gain_sizer,
        	value=self.tx_gain,
        	callback=self.set_tx_gain,
        	minimum=-30,
        	maximum=100,
        	num_steps=200,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_tx_gain_sizer)
        def _bf_weight_probe():
            while True:
                val = self.EnergyBeamforming_randphpert4_f_0.poll_complex_gain()
                try:
                    self.set_bf_weight(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (200000))
        _bf_weight_thread = threading.Thread(target=_bf_weight_probe)
        _bf_weight_thread.daemon = True
        _bf_weight_thread.start()
        self.wxgui_scopesink2_1 = scopesink2.scope_sink_f(
        	self.GetWin(),
        	title="Scope Plot",
        	sample_rate=samp_rate,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=1,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label="Counts",
        )
        self.Add(self.wxgui_scopesink2_1.win)
        self.wxgui_scopesink2_0 = scopesink2.scope_sink_f(
        	self.GetWin(),
        	title="Scope Plot",
        	sample_rate=samp_rate,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=1,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label="Counts",
        )
        self.Add(self.wxgui_scopesink2_0.win)
        self._variable_static_text_0_static_text = forms.static_text(
        	parent=self.GetWin(),
        	value=self.variable_static_text_0,
        	callback=self.set_variable_static_text_0,
        	label='variable_static_text_0',
        	converter=forms.str_converter(),
        )
        self.Add(self._variable_static_text_0_static_text)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("serial=30BC5F6", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(rx_freq, dsp_freq=0, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_source_0.set_gain(60, 0)
        self.uhd_usrp_source_0.set_antenna("RX2", 0)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(("serial=30BC5F6", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(tx_freq, dsp_freq=0, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.low_pass_filter_1 = filter.fir_filter_fff(17, firdes.low_pass(
        	1, samp_rate, 2400, 1200, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, samp_rate, 60000, 25000, firdes.WIN_HAMMING, 6.76))
        self.digital_clock_recovery_mm_xx_1 = digital.clock_recovery_mm_ff(8.33, 0.01, 0, 0.1, 0.01)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.blocks_char_to_float_0 = blocks.char_to_float(1, 1)
        self.analog_quadrature_demod_cf_0_0 = analog.quadrature_demod_cf(samp_rate/(2*math.pi*fsk_deviation_hz/8.0))
        self.EnergyBeamforming_tx_packet_source_0 = EnergyBeamforming.tx_packet_source(samp_rate, 0.040, 1500, rx_freq, tx_freq)
        self.EnergyBeamforming_multi_slicer_fi_0 = EnergyBeamforming.multi_slicer_fi(5.5, -10)
        self.EnergyBeamforming_lo_estimation_cf_0 = EnergyBeamforming.lo_estimation_cf(samp_rate, 0.00001, 1200, 512, 192)
        self.EnergyBeamforming_ekf_ff_0 = EnergyBeamforming.ekf_ff(samp_rate, 0.050, 0, (0.05, 1, 1, 38), (0.0000000225, 0.0000000225, 100))
        self.EnergyBeamforming_bfweight_cc_0 = EnergyBeamforming.bfweight_cc(bf_weight)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.EnergyBeamforming_bfweight_cc_0, 0), (self.uhd_usrp_sink_0, 0))    
        self.connect((self.EnergyBeamforming_ekf_ff_0, 1), (self.EnergyBeamforming_tx_packet_source_0, 1))    
        self.connect((self.EnergyBeamforming_ekf_ff_0, 0), (self.EnergyBeamforming_tx_packet_source_0, 0))    
        self.connect((self.EnergyBeamforming_ekf_ff_0, 3), (self.EnergyBeamforming_tx_packet_source_0, 3))    
        self.connect((self.EnergyBeamforming_ekf_ff_0, 2), (self.EnergyBeamforming_tx_packet_source_0, 2))    
        self.connect((self.EnergyBeamforming_lo_estimation_cf_0, 2), (self.EnergyBeamforming_ekf_ff_0, 2))    
        self.connect((self.EnergyBeamforming_lo_estimation_cf_0, 4), (self.EnergyBeamforming_ekf_ff_0, 4))    
        self.connect((self.EnergyBeamforming_lo_estimation_cf_0, 0), (self.EnergyBeamforming_ekf_ff_0, 0))    
        self.connect((self.EnergyBeamforming_lo_estimation_cf_0, 1), (self.EnergyBeamforming_ekf_ff_0, 1))    
        self.connect((self.EnergyBeamforming_lo_estimation_cf_0, 3), (self.EnergyBeamforming_ekf_ff_0, 3))    
        self.connect((self.EnergyBeamforming_multi_slicer_fi_0, 0), (self.blocks_char_to_float_0, 0))    
        self.connect((self.EnergyBeamforming_tx_packet_source_0, 0), (self.EnergyBeamforming_bfweight_cc_0, 0))    
        self.connect((self.analog_quadrature_demod_cf_0_0, 0), (self.low_pass_filter_1, 0))    
        self.connect((self.blocks_char_to_float_0, 0), (self.EnergyBeamforming_randphpert4_f_0, 0))    
        self.connect((self.blocks_char_to_float_0, 0), (self.wxgui_scopesink2_0, 0))    
        self.connect((self.blocks_complex_to_mag_0, 0), (self.wxgui_scopesink2_1, 0))    
        self.connect((self.digital_clock_recovery_mm_xx_1, 0), (self.EnergyBeamforming_multi_slicer_fi_0, 0))    
        self.connect((self.low_pass_filter_0, 0), (self.EnergyBeamforming_lo_estimation_cf_0, 0))    
        self.connect((self.low_pass_filter_0, 0), (self.analog_quadrature_demod_cf_0_0, 0))    
        self.connect((self.low_pass_filter_0, 0), (self.blocks_complex_to_mag_0, 0))    
        self.connect((self.low_pass_filter_1, 0), (self.digital_clock_recovery_mm_xx_1, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.low_pass_filter_0, 0))    

    def get_bf_weight(self):
        return self.bf_weight

    def set_bf_weight(self, bf_weight):
        self.bf_weight = bf_weight
        self.set_variable_static_text_0(self.bf_weight)
        self.EnergyBeamforming_bfweight_cc_0.set_bf_weight(self.bf_weight)

    def get_variable_static_text_0(self):
        return self.variable_static_text_0

    def set_variable_static_text_0(self, variable_static_text_0):
        self.variable_static_text_0 = variable_static_text_0
        self._variable_static_text_0_static_text.set_value(self.variable_static_text_0)

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self._tx_gain_slider.set_value(self.tx_gain)
        self._tx_gain_text_box.set_value(self.tx_gain)
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)
        	

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(self.tx_freq, dsp_freq=0, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_quadrature_demod_cf_0_0.set_gain(self.samp_rate/(2*math.pi*self.fsk_deviation_hz/8.0))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, 60000, 25000, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_1.set_taps(firdes.low_pass(1, self.samp_rate, 2400, 1200, firdes.WIN_HAMMING, 6.76))
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.wxgui_scopesink2_0.set_sample_rate(self.samp_rate)
        self.wxgui_scopesink2_1.set_sample_rate(self.samp_rate)

    def get_rx_freq(self):
        return self.rx_freq

    def set_rx_freq(self, rx_freq):
        self.rx_freq = rx_freq
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(self.rx_freq, dsp_freq=0, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)

    def get_fsk_deviation_hz(self):
        return self.fsk_deviation_hz

    def set_fsk_deviation_hz(self, fsk_deviation_hz):
        self.fsk_deviation_hz = fsk_deviation_hz
        self.analog_quadrature_demod_cf_0_0.set_gain(self.samp_rate/(2*math.pi*self.fsk_deviation_hz/8.0))


def main(top_block_cls=top_block, options=None):

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
