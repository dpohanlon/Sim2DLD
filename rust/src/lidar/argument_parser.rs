use std::collections::HashMap;

/// Parses command-line arguments into a HashMap of key-value pairs.
/// Flags (e.g., `--verbose`) will have an empty string as their value.
pub fn parse_args(args: Vec<String>) -> HashMap<String, String> {
    let mut args_map = HashMap::new();
    let mut iter = args.iter().peekable();

    while let Some(arg) = iter.next() {
        if arg.starts_with("--") {
            let key = arg.trim_start_matches('-').to_string();

            // Check if the next argument exists and does not start with '--' (meaning it's a value)
            if let Some(next_arg) = iter.peek() {
                if !next_arg.starts_with("--") {
                    args_map.insert(key, iter.next().unwrap().clone());
                } else {
                    // Insert the key with an empty value if it's a flag
                    args_map.insert(key, String::new());
                }
            } else {
                // Insert the key with an empty value if it's a flag
                args_map.insert(key, String::new());
            }
        }
    }

    args_map
}
