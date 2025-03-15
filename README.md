# Update: Hi HN et al.!

Other than personal "because I can" reasons, my goal with this proof-of-concept was to point at the utter state of computing platforms.

As a maker of widgets, I would like end-users to be able to do things with their brand-new widgets as painlessly as possible. This is a very relatable "user story" that has driven a lot of innovation ever since the dawn of personal computing.

However, the entire ecosystem of computers and widgets has developed into a state where there are obvious mismatches between what users intuitively expect to be possible and what is "actually" possible. The fact that this hack has been trending is just one data point in support of this -- a "security key" is "supposed to" look like a reasonably-nicely-packaged end product (i.e. not a random RPi Pico) that just does one thing and one thing only, and it's not supposed to intentionally break the rules in a way that doesn't implement any security. The fact that security keys *also* run arbitrary code, can look like anything, and can thus do arbitrary things might be quite obvious to an *extremely* technical audience who pauses to consider the implications, but it was clearly not the intention of even many of the designers involved in creating the infrastructure this hack leverages.

Other ideas which also poke at this problem include the [USB Rubber Ducky](https://shop.hak5.org/products/usb-rubber-ducky) and the [O.MG Cable](https://shop.hak5.org/products/omg-cable). The "Universal" part of USB has both advantages and disadvantages! This is something which I alluded to at the end of this README -- there is currently no good way for *either* humans or computers to easily and reliably tell whether a USB device is working against the user's interests (in simplified terms, "evil"), in support of them ("good"), or simply as a consequence of larger forces and emergent behaviors (a huge category which overlaps most of the former two).

This brings us to WebUSB and the growth and dominance of the Web as a platform. Without fully litigating whether "cross-platform" applications are a good idea or not, the Web is *by far* the easiest way for somebody to deliver software to run on somebody else's computer. I and other developers no longer need to learn the intricacies of every single target platform. As a consequence, I and other developers no longer need to learn the intricacies, the *conventions*, and the *expectations* of every single target platform.

If the popularity of retrocomputing that I am seeing in my little corner of the zeitgeist is any indication, we've lost something important along the way.

In conclusion, I want to see discussions being had which move beyond the basics of "Why won't Firefox implement WebUSB? Is it going to lose out even further to Chrome?" and more towards discussions about intentionally curating healthy platforms and ecosystems. This extends beyond the Web and includes computing in all forms (whether on desktops and laptops, tablets and phones, or much-less-visible automation such as "IoT" and "smart home" devices). Platforms need to be both healthy for developers, so that they enjoy building software for them, and healthy for users, so that they understand what is happening and can engage with computing in a way which empowers them.

I am sure that there are people who have more experience and plenty more to say on these topics than I do, and I look forward to hearing these discussions!

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

Although it is _sometimes_ possible to tell whether an ECDSA signature was actually calculated "properly" rather than being some numbers we just made up (see Issue #1), there isn't a good reason for anybody other than the relying party to perform anything beyond basic validity checks. Chrome appears to check whether the numbers in the signature are actually in the range from 0 to $n$, but Firefox doesn't check even that.

As a result, we can just generate some dummy ASN.1 and then put the data we actually want to send inside of it. In order to reliably get around Chrome's basic validity checks, we just waste the first byte of each number with the value 0x7f. This will result in numbers which are always positive and less than $n$. The entire software stack up to browser JavaScript will pass these "valid-enough" numbers straight through.

Finally, because "access to USB devices" is politically contentious but "make users more secure" has very broad political support across the entire browser industry, this capability is widely supported without requiring extraneous setup, configuration, nor prompting.

## Is this a security vulnerability?

No.

This _cannot_ be used to access arbitrary USB devices. It only works with devices which are _intentionally_ breaking the rules. In essence, this is an intentionally vulnerable device.

_However_, it is known that the security model around USB devices is generally... questionable on most platforms. Plugging in a malicious USB device allows it to do anything that you yourself can do with devices such as a keyboard or a mouse.

Do not plug arbitrary unknown devices into your computer (or your phone, etc.).
