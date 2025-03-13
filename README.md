# We don't need no stinkin' WebUSB!

It turns out that there is a way for a web page to access USB devices *without* requiring WebUSB and its associated political disagreements! Not only that, a device can intentionally design itself to bypass all of the user consent requirements.

![demo video](demo.gif)

## Quick demo

Load [u2f-hax.uf2](u2f-hax.uf2) onto a Raspberry Pi Pico (RP2040 version), and then load [index.html](index.html) from either localhost or another secure context.

The "On!" and "Off!" buttons will toggle the LED, and the state of pin `GP22` will be regularly updated on the page (you can conveniently short it to the adjacent GND pad with a piece of wire or metal).

## How?!

The Pico is programmed to emulate a [U2F](https://en.wikipedia.org/wiki/Universal_2nd_Factor) dongle (i.e. a physical two-factor security key). However, instead of performing any security functions, arbitrary data is smuggled in the "key handle" and signature of `U2F_AUTHENTICATE` messages. As long as the key handle starts with 0xfeedface, the Pico instantly "confirms" user presence and returns data.

## Why is this possible?

By design, the U2F key handle is an opaque blob of data which is conceptually "owned by" the security dongle. It is supposed to be returned by the dongle as a result of a registration, stored as-is by the relying party, and then given as-is back to the security dongle when authenticating.

One reason this key handle functionality exists is to enable an unlimited number of websites to be associated with a particular  low-cost dongle with very limited memory. This hypothetical dongle stores a unique "master" encryption key internally. When a new registration is created, it creates a new public/private key pair, returns the public key, encrypts the private key with the "master" key, and *returns the encrypted private key as the key handle*. No matter how many registrations are created, the dongle does not have to be responsible for storing the keys associated with them. When the key handle is passed back to the dongle during an authentication, the dongle just unwraps the private key using its master key.

In order to _allow for_ all of these low-cost designs without _mandating_ any particular internal algorithms, the key handle is treated as opaque, and so we can abuse it to smuggle arbitrary data.

In order to _return_ data, we need to somehow smuggle it as an ECDSA signature. An ECDSA signature is a tuple of two numbers $(r, s)$, where each of the numbers is calculated $\mod n$, where $n$ is the _order_ of the elliptic curve base point. This basically means any value from 0 up to 0xffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632551 (the order of the secp256r1 base point). These numbers are then packed into some ASN.1.

It turns out that it is either very difficult or outright impossible (real cryptographers feel free to submit an appropriate proof) to verify whether or not these numbers were indeed calculated "properly," especially without access to the private key (which is only accessible to the dongle). This means that we can just _make something up_, and the entire software stack up to browser JavaScript will pass it straight through.

In order to do this, we just generate some dummy ASN.1 and then put the data we want to send inside of it. The only important thing is to make sure that the numbers are actually in the proper range (Chrome checks, but Firefox does not). To do so, we just waste the first byte of each number with the value 0x7f. This will result in numbers which are always positive and less than $n$.

Finally, because "access to USB devices" is politically contentious but "make users more secure" has very broad political support across the entire browser industry, this capability is widely supported without requiring extraneous setup, configuration, nor prompting.

## Is this a security vulnerability?

No.

This _cannot_ be used to access arbitrary USB devices. It only works with devices which are _intentionally_ breaking the rules. In essence, this is an intentionally vulnerable device.

_However_, it is known that the security model around USB devices is generally... questionable on most platforms. Plugging in a malicious USB device allows it to do anything that you yourself can do with devices such as a keyboard or a mouse.

Do not plug arbitrary unknown devices into your computer (or your phone, etc.).
