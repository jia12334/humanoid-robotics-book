# Data Model for Physical AI and Humanoid Robotics Book

## Entities

- **Chapter**:
  - `id`: Unique identifier
  - `title`: Chapter title
  - `content`: Markdown content of the chapter
  - `module_id`: Reference to parent module
  - `order`: Order within the module

- **Module**:
  - `id`: Unique identifier
  - `title`: Module title
  - `description`: Module description
  - `order`: Order within the book

## Relationships

- A Module has many Chapters.

## Validation Rules

- Chapter titles must be unique within a module.
- Module titles must be unique.
