class HDLCforPPP (object):
    _pendingBytes = None
    """A string comprising bytes that have been read from the input
    but not yet split into frames."""
    
    _pendingPackets = None
    """A list of packets that are ready for consumption."""

    _FlagSequence = chr(0x7e)
    """The binary sequence that begins and ends each frame."""
    
    _AddressControlPrefix = struct.pack('BB', 0xff, 0x03)
    """The binary sequence representing the address and control fields
    recognized for this variant of HDLC.  The only valid address is
    the All-Stations address, and the only valid control is the
    Unnumbered Information command with the Poll/Final bit set to
    zero."""

    _ControlEscape = chr(0x7d)
    """The binary sequence introducing escaped data.  The following
    character was exclusive-or'd with the _ControlModifier prior to
    insertion into the octet stream."""
    
    _ControlModifier = 0x20
    """The binary value exclusive-or'd with a data octet to prevent it
    from being mis-interpreted as a DCE or HDLC command."""

    FErr_NullFrame = 'NullFrame'
    FErr_MissingFlagSequence = 'MissingFlagSequence'
    FErr_EscapeEscape = 'EscapeEscape'
    FErr_UnterminatedEscape = 'UnterminatedEscape'
    FErr_FailedCRC = 'FailedCRC'
    FErr_MissingACF = 'MissingACF'

    _txCompressAddressControl = False
    _frameCheckSequenceHelper = FrameCheckSequence16

    _sendingACCM = None
    """The set of characters that should be octet-stuffed when sending
    data.  The default is all ASCII values less than 0x20, plus the
    flag sequence (0x7e) and control escape (0x7d) characters.  Use of
    the L{augmentSendingACCM} method allows additional characters,
    including non-control characters, to be escaped as well, if
    necessary.  If ACCM negotiation occurs, control characters not in
    the negotiated set are dropped from this set, even if specified by
    L{augmentSendingACCM}.

    @note: This is the remote value of the LCP ACCM option."""

    _needLeadingFlagSequence = True

    _receivingACCM = None
    """The set of ASCII control characters that the sender is expected
    to escape.  If any such character is encountered unescaped in an
    incoming packet, it is dropped on the assumption it was introduced
    by the DCE.  This set can be changed as a result of option
    negotiation, but only ASCII control characters excluding DEL may
    be specified.

    @note: This is the local value of the LCP ACCM option."""

    _receivingACCMMustInclude = set()
    """A set of characters that we know the sender must escape.  This
    is only used if the remote negotiates a non-default ACCM.  Only
    characters with an ordinal value below 0x20 may appear in this
    set, since only those characters are negotiable."""

    _requireACCMNegotiation = True
    
    def setReceivingACCMMustInclude (self, accm_set):
        """Set the local end to request ACCM configuration and to
        incorporate the control characters found in the given set.

        Control characters are ASCII octets with a value less than 32
        decimal.  By default, the L{_receivingACCMMustInclude} set is
        empty, and L{_requireACCMNegotiation} is C{True} so normally
        no control characters are escaped.  Only use this if you know
        the link can introduce control characters that were not
        transmitted.
        """
        self._receivingACCMMustInclude = set()
        for c in accm_set:
            if ord(c) < self._ControlModifier:
                self._receivingACCMMustInclude.add(c)
        self._requireACCMNegotiation = True

    _negotiatedACCM = False
    """C{True} iff option negotiation configured a receiving ACCM."""

    def validReceivingACCM (self, accm_word):
        """Return a word representing the union of the given word and
        those characters that this end knows must be escaped.

        See L{_receivingACCMMustInclude}.
        """
        accm = self._WordToACCM(accm_word, with_fsce=False)
        acceptable_accm = self._receivingACCMMustInclude.union(accm)
        if accm != acceptable_accm:
            return self._ACCMToWord(acceptable_accm)
        return accm_word

    def updateReceivingACCM (self, accm_word, validate=False, negotiated=True):
        """Process an ACCM that has been accepted by the remote.

        When the local node receives a Configure-Ack for the ACCM
        option, it uses the negotiated set as the new receiving ACCM."""

        self._receivingACCM.clear()
        self._receivingACCM.update(self._WordToACCM(accm_word, with_fsce=False))
        self._negotiatedACCM = negotiated

    def updateSendingACCM (self, accm_word):
        """Process a remote ACCM that was accepted.

        When the local node transmits a Configure-Ack that accepts an
        ACCM, it removes from its sendingACCM any control characters
        that are not part of the negotiated set."""
        self._sendingACCM = self._sendingACCM.copy()
        accm_set = self._WordToACCM(accm_word)
        for o in xrange(32):
            c = chr(o)
            if not (c in accm_set):
                self._sendingACCM.discard(c)

    @classmethod
    def _Init_ (cls):
        """Class-level initialization that requires a class reference."""
        cls._SendingACCM = cls._WordToACCM(0xFFFFFFFF)
        cls._ReceivingACCM = cls._WordToACCM(0xFFFFFFFF, with_fsce=False)

    @classmethod
    def _WordToACCM (cls, uint32, with_fsce=True):
        """Return the set of characters Async-Control-Character-Map
        value.

        Converts a 32-bit integer into the set of ASCII control
        characters identified by set bits in the integer, starting
        with bit zero (least-significant bit) representing ASCII NUL.

        @keyword with_fsce: If C{True} (default), the flag sequence
        and control escape characters are added to the set."""
        accm = set()
        if with_fsce:
            accm.update([ cls._FlagSequence, cls._ControlEscape ])
        for bp in xrange(32):
            if uint32 & (1 << bp):
                accm.add(chr(bp))
        return accm

    @classmethod
    def _ACCMToWord (cls, accm):
        """Return a 32-bit value representing the members of the set
        with an ordinal value less than 32.

        Only those values are negotiable async control character map
        elements."""
        uint32 = 0
        for ch in accm:
            o = ord(ch)
            if 0x20 > o:
                uint32 |= (1 << o)
        return uint32

    def __init__ (self, **kw):
        self._sendingACCM = self._SendingACCM.copy()
        self._receivingACCM = self._ReceivingACCM.copy()
        self._pendingBytes = ''
        self._txCompressAddressControl = kw.get('compress_ac', self._txCompressAddressControl)
        self._pendingPackets = []

    def putBytes (self, bytes):
        """Feed the provided bytes into the framer for de-framing.

        If enough data is present to identify one or more frames,
        those frames are extracted, converted to packets, and made
        available through L{generatePackets}."""
        
        self._pendingBytes += bytes
        while True:
            i1 = self._pendingBytes.find(self._FlagSequence)
            if 0 > i1:
                break
            i2 = self._pendingBytes.find(self._FlagSequence, i1+1)
            if 0 > i2:
                break
            frame = self._pendingBytes[i1+1:i2]
            if 0 < len(frame):
                packet = self.convertFrame(frame, has_fs=False)
                if packet is not None:
                    self._pendingPackets.append(packet)
                else:
                    print 'RX %s DROPPED %s:' % (self._frameError, binascii.hexlify(frame),)
            self._pendingBytes = self._pendingBytes[i2:]

    def getPacket (self):
        """Return the next ready packet, or C{NOne} if no packets are ready."""
        if self._pendingPackets:
            return self._pendingPackets.pop(0)
        return None

    def generatePackets (self):
        """A generator for ready packets."""
        while self._pendingPackets:
            yield self._pendingPackets.pop(0)

    def convertFrame (self, frame, has_fs=None, compress_ac=None, has_fcs=True):
        """Strip all HDLC-related material from the given string.

        All octet stuffing is reversed.    A leading address and control
        prefix is stripped, if present.  The validated, stripped, and
        unstuffed string is returned, unless something went wrong in
        which case C{None} is returned instead.

        @param frame: The string to be converted, with leading and
        trailing flag sequences already removed.  If C{frame} is
        C{None}, the value C{None} is immediately returned.

        @keyword has_fs: If C{True}, the incoming frame must have flag
        sequence characters as prefix and suffix characters.  If
        C{None} (default), the flag sequence is removed if present at
        the front and/or the back, on the assumption that the packet
        may have taken advantage of a previously transmitted flag
        sequence and elided its initial one.  If C{False}, no
        prefix/suffix stripping is done.

        @keyword compress_ac: If C{False}, the frame is expected to
        have the address and control field bytes, which are stripped
        before it is returned.  If C{True}, no check is made for
        address and control field bytes.  If C{None} (default), the
        address and control fields are stripped if present.

        @keyword has_fcs: If C{True} (default), the check sequence is validated.

        @return: The converted string, or C{None} if something went
        wrong with the conversion.
        """
        if frame is None:
            self._frameError = self.FErr_NullFrame
            return None
        self._frameError = None
        if has_fs is not False:
            fs0 = frame.startswith(self._FlagSequence)
            fs1 = frame.endswith(self._FlagSequence)
            if has_fs and not (fs0 and fs1):
                self._frameError = self.FErr_MissingFlagSequence
                return None
            if fs0 and fs1:
                frame = frame[1:-1]
            elif fs0:
                frame = frame[1:]
            elif fs1:
                frame = frame[:-1]
        in_escape = False
        unstuffed = []
        for ch in frame:
            if self._ControlEscape == ch:
                if in_escape:
                    # Abort frame
                    self._frameError = self.FErr_EscapeEscape
                    return None
                in_escape = True
                continue
            if in_escape:
                ch = chr(ord(ch) ^ self._ControlModifier)
                in_escape = False
            elif (ord(ch) < self._ControlModifier) and (ch in self._receivingACCM):
                # Drop characters possibly introduced by DCE
                continue
            unstuffed.append(ch)
        if in_escape:
            self._frameError = self.FErr_UnterminatedEscape
            return None
        unstuffed = ''.join(unstuffed)
        if has_fcs:
            unstuffed = self._frameCheckSequenceHelper.PacketIfGood(unstuffed)
        if unstuffed is None:
            self._frameError = self.FErr_FailedCRC
            return None
        if not compress_ac:
            if unstuffed.startswith(self._AddressControlPrefix):
                unstuffed = unstuffed[2:]
            elif False == compress_ac:
                self._frameError = self.FErr_MissingACF
                return None
        return unstuffed
        
    def framePacket (self, packet, use_fs=None, compress_ac=None, append_fcs=True):
        """Frame the packet.

        The standard address and control fields are optionally
        prepended.  The frame check sequence is optionally computed
        and appended.  Characters in the resulting packet are
        converted in accordance with the sending ACCM.  Optionally,
        leading and trailing flag sequence characters and the frame
        check sequence are added, and the resulting string returned.

        @keyword use_fs: If C{True}, the flag sequence character is
        the first and the last character in the returned frame.  If
        C{None} (default), the flag sequence character is the last
        character in the returned frame, but the initial flag sequence
        character is elided if the last framed packet ended with a
        flag sequence character.

        @keyword compress_ac: The address and control sequence is used
        only if C{compress_ac} is C{False}.  If C{None} (default), the
        value for C{compress_ac} is taken from the framer
        configuration.

        @keyword append_fcs: If C{True} (default), the frame check
        sequence is appended in its proper location.
        """

        framed = []
        if use_fs or ((use_fs is None) and self._needLeadingFlagSequence):
            framed.append(self._FlagSequence)
        if append_fcs:
            packet += self._frameCheckSequenceHelper.Calculate(packet, pack=True)
        for ch in packet:
            # Note RFC1662:7.1 compatibility with 3309:1991/Amendment
            # 2 (8-bit transparency)
            if (ch in self._sendingACCM) or (chr(ord(ch) & 0x7F) in self._sendingACCM) or ((not self._negotiatedACCM) and (chr(0x7F) == ch)):
                framed.append(self._ControlEscape)
                framed.append(chr(ord(ch) ^ self._ControlModifier))
            else:
                framed.append(ch)
        self._needLeadingFlagSequence = (1 < len(packet)) or ((1 == len(packet)) and (self._FlagSequence != packet[0]))
        if use_fs is not False:
            framed.append(self._FlagSequence)
            self._needLeadingFlagSequence = False
        framed = ''.join(framed)
        return framed

# Perform class-level initialization that could not be done inline
HDLCforPPP._Init_()