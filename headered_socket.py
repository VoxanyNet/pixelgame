import socket


class InvalidHeader(Exception):
    pass


class Disconnected(Exception):
    pass


class PayloadTooLarge(Exception):
    pass


# a socket that has the ability to add headers
class HeaderedSocket(socket.socket):

    def send_headered(self, data, header_size=7):
        # construct a payload with header from bytes

        # get number of bytes json string uses
        data_size = len(data)

        # the amount of data that has been acknowledged by the client to have been sent
        # somtimes we need to resend data if the client does not acknowledge it
        # im not sure why sockets don't do this automatically
        # in my case the clients receive buffer was filling up
        sent_data = 0

        # if the number used to represent the length of the payload is over 7 characters we cant trasmit it
        if len(str(data_size)) > header_size:
            raise PayloadTooLarge(f"Payload header cannot be more than {header_size} characters")

            return

        # create header containing payload size - this header must always be the same size: 7 characters
        # header contains the number of bytes that the json payload is
        # if the number of bytes requires less than 7 characters to represent, we use leading zeros
        header = f"{(header_size - len(str(data_size))) * '0'}{data_size}"  # 7 - len(str(payload_size_string)) fills in unused digits with 0s

        header_bytes = bytes(header, "utf-8")

        # final payload with header and data
        headered_data = header_bytes + data

        # keep sending until
        while sent_data != len(headered_data):
            sent_data += self.send(
                headered_data[sent_data:]  # send headered data starting at what we've sent so far
            )

    def recv_headered(self, header_size=7):

        # read the header

        header = self.recv(header_size).decode("utf-8")

        if header == "":
            # the socket will return a blank string when the client has sent FIN packet
            raise Disconnected("Remote socket disconnected")

            return

        # convert header string to int
        try:
            payload_length = int(header)

        # if we cannot convert to int
        except ValueError:
            raise InvalidHeader(f"Sender sent header {header}, which is invalid")

        # represents the final constructed data sent
        constructed_data = bytearray()

        # until we have received the full payload, we do not stop reading the buffer
        while len(constructed_data) != payload_length:
            # read what is currently in the buffer and add it to the constructed_data byte array
            new_data = self.recv(payload_length - len(constructed_data))

            constructed_data.extend(new_data)

        return constructed_data

    # accept() needs to be redefined to return PayloadSockets instead of default ones
    def accept(self):

        fd, addr = self._accept()

        # create a headered socket instead of a normal socket
        sock = HeaderedSocket(self.family, self.type, self.proto, fileno=fd)

        # blocking sockets return blocking and non blocking return non blocking
        if self.timeout == 0 or None:
            sock.settimeout(0)

        return sock, addr
