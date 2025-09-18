import face_recognition
import os
import pickle
import rclpy # Added for ROS 2 package path finding

def main():
    """
    Scans the known_faces directory, calculates face encodings for each person,
    and saves them to a pickle file for fast loading.
    """
    # --- Configuration ---
    KNOWN_FACES_DIR = "known_faces"
    ENCODINGS_FILE = "encodings.pkl"

    print("Starting to scan the known_faces directory...")
    known_face_encodings = []
    known_face_names = []

    if not os.path.exists(KNOWN_FACES_DIR):
        print(f"❌ ERROR: The '{KNOWN_FACES_DIR}' directory was not found here.")
        print("Please create it and add sub-folders for each person with their pictures.")
        return

    for name in os.listdir(KNOWN_FACES_DIR):
        person_dir = os.path.join(KNOWN_FACES_DIR, name)

        if not os.path.isdir(person_dir):
            continue

        print(f"Processing images for: {name}")
        for filename in os.listdir(person_dir):
            image_path = os.path.join(person_dir, filename)
            try:
                image = face_recognition.load_image_file(image_path)
                face_encodings = face_recognition.face_encodings(image)
                if face_encodings:
                    known_face_encodings.append(face_encodings[0])
                    known_face_names.append(name)
                else:
                    print(f"⚠️ Warning: No face found in {filename}, skipping.")
            except Exception as e:
                print(f"❌ Error processing {filename}: {e}")

    print("\nSaving encodings to disk...")
    data_to_save = {"encodings": known_face_encodings, "names": known_face_names}

    with open(ENCODINGS_FILE, "wb") as f:
        pickle.dump(data_to_save, f)

    print(f"✅ Success! Encodings saved to '{ENCODINGS_FILE}'.")
    print(f"Total of {len(known_face_encodings)} images from {len(set(known_face_names))} people processed.")

if __name__ == '__main__':
    main()