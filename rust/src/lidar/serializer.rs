use ndarray::Array2;
use serde::ser::{Serialize, Serializer};
use serde_json::to_writer;
use std::fs::File;
use std::io::BufWriter;
use std::io::Result as IoResult;

// Wrap Array2 in a new struct
pub struct SerializableArray2<T> {
    pub array: Array2<T>,
}

// Implement Serialize for the wrapper struct
impl<T: Serialize + std::clone::Clone> Serialize for SerializableArray2<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Convert Array2 into a serializable Vec<Vec<T>>
        let vec_of_vecs: Vec<Vec<T>> = self
            .array
            .rows()
            .into_iter()
            .map(|row| row.to_vec())
            .collect();

        // Serialize the Vec<Vec<T>>
        vec_of_vecs.serialize(serializer)
    }
}

// A function to write a serializable object to a JSON file
pub fn write_to_json<T: Serialize>(filename: &str, data: &T) -> IoResult<()> {
    let file = File::create(filename)?; // Open a file in write mode
    let writer = BufWriter::new(file); // Create a buffered writer for efficient writing
    to_writer(writer, data)?; // Serialize the data to JSON and write it to the file
    Ok(())
}
