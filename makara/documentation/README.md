# MAV Simulator Documentation

This directory contains the Quarto book documentation for the MAV Simulator project.

## Quick Start

To build and view the documentation:

1. Install Quarto from https://quarto.org/docs/getting-started/installation.html
2. Run the provided scripts:
   ```bash
   # To build the documentation
   ./build_docs.sh
   
   # To view the documentation with live preview
   ./view_docs.sh
   ```

## Documentation Structure

- `_quarto.yml` - Configuration file for the Quarto book
- `index.qmd` - Home page of the documentation
- `intro.qmd` - Introduction to the MAV Simulator
- Core modules documentation:
  - `vessel.qmd` - Vessel class documentation
  - `kinematics.qmd` - Kinematics module documentation
  - `control.qmd` - Control module documentation
  - `sensors.qmd` - Sensors module documentation
  - `world.qmd` - World class documentation
- Utilities documentation:
  - `simulation.qmd` - Simulation modules documentation
  - `hydrodynamics.qmd` - Hydrodynamics calculation documentation
- ROS2 integration:
  - `ros2_integration.qmd` - ROS2 interface documentation
- `references.qmd` - References and build instructions

## Adding Content

To add new content to the documentation:

1. Create a new `.qmd` file in this directory
2. Add the file to the `chapters` section in `_quarto.yml`
3. Run `./build_docs.sh` to rebuild the documentation

## Output Formats

The documentation can be rendered in multiple formats:

- HTML (default) - `quarto render`
- PDF - `quarto render --to pdf`
- EPUB - `quarto render --to epub`

## Contributing

When contributing to the documentation, please follow these guidelines:

1. Use consistent formatting and style
2. Include code examples where appropriate
3. Provide diagrams for complex concepts
4. Add references to external resources

## License

This documentation is provided under the same license as the MAV Simulator project. 