# ADASISv2 serialization and deserialization library

This library implements the [ADASIS v2 protocol specification version 2.0.5.0](https://adasis.org/wp-content/uploads/sites/10/2024/08/200v2.0.5-D2.2-ADASIS_v2_Specification.0.pdf) (March 2024).

The library is the first open source implementation of the specification I know of and is experimental. See [LICENSE.md] for the applicable license.

## ADASIS v2

ADASIS v2 is a communication protocol with an 8 byte packet size  used in certain automotive and mobility
systems to transmit map navigation route and environment information from a central navigation provider
to navigation data clients, primarily with a driver assistance application intent. The 8 byte packet size
was originally chosen to be compatible with the Controller Area Network (CAN) 2.0 packet size.

Each ADASIS v2 packet contains a 5-bit header identifying the message type and a sequence id. Sequence
counters are specific to each message type. The ADASIS v2 standard defines the following message types:

* META-DATA - Describing general data source information and the current country and region
* POSITION - Describes the estimated vehicle speed, map heading and vehicle progression point (in meters) along a selected arbitrary navigation path
* SEGMENT - Describes the high-level road environment (e.g. speed limits) along segments of the current navigation path
* STUB - Describes intersections and related driving guidances along the current navigation path, along with high-level road environment information
* PROFILE SHORT - Describes relative road geometry, road access, road surface information etc along the current navigation path depending on the message subtype
* PROFILE LONG - Describes absolute road geometry, and other information along the current navigation path depending on the message subtype.

Possible applications include:
* Look-ahead planning transmission and torque control using upcoming road slope and maneuver information
* SD Map Navigation context provision (visually, audibly or textually) to the driver
* SD Map Navigation support for Hands-off SAE L2 and L3 autonomous driving systems

Multiple navigation information service providers support ADASIS information transmission, including but not limited to:
* [TomTom](https://www.tomtom.com/)
* [HERE Technologies](https://www.here.com/)
* [MapBox](https://www.mapbox.com/)

The v2 specification is superseded by the more freeform ADASIS v3 specification that is limited to only the
Franca Interface Description Language (IDL) message descriptions, and so is not implementable as a library per se;
each v3 integration requires bespoke signal and protocol integration work unlike in ADASIS v2.

## Horizon Reconstruction

The ADASIS v2 protocol is intended to be used together with a so-called Horizon Reconstructor that statefully pieces together
the larger "puzzle" from the information pieces transmitted using the ADASIS v2 protocol. A Horizon
Reconstructor is not provided with this library, but implementing one is not particularly difficult for a single
navigation path. An important consideration is that the implementer needs to consider aligning information along a unidimensional
abstract navigation path with the real-world geodetic geometry information to allow e.g. a visual reconstruction of
the information.

## Building and usage

### Build and test

*Using Bazel*

Build the library and the example application using `bazel build //rust/...`
Run the example application using `bazel run //rust:usage_example`.

*Using Cargo*

in the `rust/` subdirectory, run `cargo build`. The test program executable will can be found under the created `target/` directory tree.

### Use in other projects

It is recommended that you import the library via Bazel, as the library is bzlmod (MODULE.bazel) compatible out of the box.
This accomplished by adding the following to your respective MODULE.bazel file

```
bazel_dep(name = "adasisv2")
git_override(
    module_name = "adasisv2",
    remote = "https://github.com/kashiwado/adasisv2.git",
    branch = "main",
)
```
