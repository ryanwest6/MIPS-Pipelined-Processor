View the Verilog implementation of the processor:

`mips_basic_pipeline.v` is a basic implementation without forwarding actually built in. 

[Basic Processor](mips_pipeline_newest.srcs/sources_1/new/mips_basic_pipeline.v)


`mips_forwarding_pipeline.v` includes a 5-stage pipeline with full forwarding from the ALU and data memory, as well as detection and correction of various hazards that may occur.

[Forwarding Processor](mips_pipeline_newest.srcs/sources_1/new/mips_forwarding_pipeline.v)
