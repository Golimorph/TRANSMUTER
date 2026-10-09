# Code Structure

Structure orchestration methods such as threeAxisSpiralAlgorithm::generate() as a concise top-level narrative, delegating each stage to well-named helpers at successively lower abstraction levels while keeping representation-specific operations behind interfaces.

Keep one implementation class per file, with matching class and file names. Keep public contracts in
`if` directories and concrete implementations in their owning module's `inc`/`src` directories.
Consumers of another module must use its public interface, not its private helpers. Prefer existing
geometry operations over duplicating material-removal state in another abstraction.
