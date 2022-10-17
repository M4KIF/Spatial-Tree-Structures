#pragma once

//Custom Libraries

//Default Libraries
#include<list>
#include<queue>
#include<memory>
#include<iostream>
#include<algorithm>

//Game files
#include<Engine/Collisions/AABB.h>

//Macros
#define NUMBER_OF_CHILDREN 4


/*
* This QuadTree implementation is meant to be used with the game engine, so
* here I will also add the template specialisation used specifically for the chunk map. 
* It will be made similarly to the QuadTree and QuadTreeContainer. So essentially, the map will be a wrapper
* consisting of the Quadtree and a few QuadTrees.
*/


namespace DataStructures {

	template<typename T>
	class QuadTree
	{
		//Alias for the children coordinates
		using ChildrenBoxes = std::array<Collisions::AABB, 4>;

		//Trick for storing the data about Children
		enum class Children : unsigned char {
			O1 = 0x01, // = 0b00000001 -> 1 << 1
			O2 = 0x02, // = 0b00000010 -> 1 << 2
			O3 = 0x04, // = 0b00000100 -> 1 << 3
			O4 = 0x08, // = 0b00001000 -> 1 << 4
		};


		//Intelligently calculates the BB's for all needed Children
		void calculate_bounding_box(Collisions::AABB& BoundingBox, Children Octant);

		//Speaks for itself
		bool is_leaf_node(void);

		//
		std::array<std::shared_ptr<QuadTree<T>>, 4>& access_children();

		//
		void collect_items(std::list<std::pair<T, Collisions::AABB>>& items);

		//Set of minimal recursive functions that just do their tasks, without tree safety
		void recursive_subdivide(void); //OK
		void recursive_dfs(Collisions::AABB& area, std::list<T>& items); //OK
		Trees::Location<T> recursive_insert(T object, Collisions::AABB area); //OK
		void recursive_resize(Collisions::AABB& area); //OK

	protected:

		//The dimensions of the tree
		Collisions::AABB m_Position;
		size_t m_LeafNodeSide;
		size_t m_MinimumDimensions;

		// Depth checking
		size_t m_MaxDepth;
		size_t m_Depth = 0;

		// A clever way of knowing whether the Children are active
		// if the value is shifted 8 times to the left, that means, that all of the Children have been set
		unsigned char m_ActiveChildren;

		// The bounds of the potential Children
		ChildrenBoxes m_ChildrenBounds;

		// The Children themselves, will be made with the use of a bounds calulating function
		std::array<std::shared_ptr<QuadTree<T>>, 4> m_Children;

		// The flag set
		bool m_IsLeaf = false;
		bool m_NodeReady = false;

		//Item that the node is storing. Can become anything that the programmer wants it to
		std::list<T> m_Item;

	public:

		/*//////////////
		* Initialisation
		*///////////////

		QuadTree();
		QuadTree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions);
		QuadTree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions, size_t Depth);
		~QuadTree();

		/*
		* Dimensions & Position
		*/

		size_t min_dimensions();
		size_t leaf_node_side_length();
		Collisions::AABB& aabb();
		ChildrenBoxes children_positions();
		void resize(Collisions::AABB area);

		/*////////
		* Capacity
		*/////////

		size_t size();
		size_t max_size();
		size_t depth();
		size_t max_depth();
		bool empty();

		/*//////////////
		* Element access
		*///////////////

		void dfs(Collisions::AABB& area, std::list<T>& items); //TODO
		void bfs(Collisions::AABB& area, std::list<T>& items); //TODO
		bool contains(Collisions::AABB& area); //OK
		void erase_area(Collisions::AABB& area, std::list<T>& items);
		std::list<T> access_elements();

		/*/////////
		* Modifiers
		*//////////

		Trees::Location<T> insert(T object, Collisions::AABB area); //OK
		void clear(); //OK 

		/*//////////////
		* Space altering
		*///////////////

		//TODO: To redesign this function for it to suit the chunking system better. To specify the template for the chunks
		void shift(size_t leaf_nodes, Coordinates::Directions direction, std::list<std::pair<T, Collisions::AABB>>& returned_data);
	};



	/*
	* ///////////////////////
	* /		Definitions     /
	* ///////////////////////
	*/


	//Default Constructor
	template<typename T>
	QuadTree<T>::QuadTree()
	{
		// Does nothing, because the QuadTree doesn't have the bounding space defined
	}


	//Area setting constructor, should be always first
	template<typename T>
	QuadTree<T>::QuadTree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions) :
		m_Position(BoundingBox)
	{
		//Setting max depth available for the Tree
		m_MaxDepth = MaxDepth;
		m_MinimumDimensions = MinimumDimensions;

		//Calculating the side length
		m_LeafNodeSide = m_Position.dimensions().x / std::pow(2.0, m_MaxDepth);

		//Every QuadTree starts as a leaf node before any subdivisions
		m_IsLeaf = true;
		m_NodeReady = true;

		//Proceeds to subdivision
		recursive_subdivide();
	}


	//Area setting constructor, that takes a depth param, used in subdivision
	template<typename T>
	QuadTree<T>::QuadTree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions, size_t Depth) :
		m_Position(BoundingBox), m_Depth(Depth)
	{
		//Every QuadTree starts as a leaf node before any subdivisions
		m_IsLeaf = true;
		m_NodeReady = true;

		//Sets the current level
		m_Depth = Depth;
		m_MaxDepth = MaxDepth;
		m_MinimumDimensions = MinimumDimensions;

		//Calculating the side length
		m_LeafNodeSide = m_Position.dimensions().x / std::pow(2.0, m_MaxDepth);

		//Proceeds to subdivision
		recursive_subdivide();
	}


	//
	template<typename T>
	QuadTree<T>::~QuadTree()
	{}


	/*////////////////////
	* /     Capacity     /
	*/////////////////////

	template<typename T>
	size_t QuadTree<T>::leaf_node_side_length()
	{
		return m_LeafNodeSide;
	}


	template<typename T>
	Collisions::AABB& QuadTree<T>::aabb()
	{
		return m_Position;
	}


	template<typename T>
	std::array<Collisions::AABB, 4> QuadTree<T>::children_positions()
	{
		return m_ChildrenBounds;
	}


	template<typename T>
	size_t QuadTree<T>::size()
	{
		size_t count = 0;

		//Checks if the item is allocated
		if (!m_Item.empty())
		{
			//If yes, then adds one to count
			count += m_Item.size();
		}
		//Iterates recursively through all of the Children
		for (int i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			if (m_Children[i])
			{
				count += m_Children[i]->size();
			}
		}

		//Returns the final value and recursive iteration values
		return count;
	}


	template<typename T>
	size_t QuadTree<T>::max_size()
	{
		//Returns a theoretical maximum size, which equals to the maximum possible nodes
		return std::pow(NUMBER_OF_CHILDREN, m_MaxDepth);
	}


	template<typename T>
	size_t QuadTree<T>::min_dimensions()
	{
		return m_MinimumDimensions;
	}


	template<typename T> inline
		size_t QuadTree<T>::depth()
	{
		return m_Depth;
	}


	template<typename T> inline
		size_t QuadTree<T>::max_depth()
	{
		return m_MaxDepth;
	}


	template<typename T>
	void QuadTree<T>::resize(Collisions::AABB area)
	{
		//The tree has to be built a new, data is invalidated
		recursive_resize(area);
	}


	template<typename T>
	bool QuadTree<T>::empty()
	{
		//If there are no Children in the root node, then the tree is empty
		if (!m_Children[0]) return true;
	}


	/*////////////////////
	* / Element Access   /
	*/////////////////////


	//Searches for a given area inside the tree
	template<typename T>
	void QuadTree<T>::dfs(Collisions::AABB& area, std::list<T>& items)
	{
		//This can go deep into the recursion
		recursive_dfs(area, items);
	}


	//Searches for a given area inside the tree
	template<typename T>
	void QuadTree<T>::bfs(Collisions::AABB& area, std::list<T>& items)
	{
		//Lamda for asigning the children to the queue
		auto asign_children = [](std::list<std::shared_ptr<QuadTree<T>>>& temp_queue, std::array<std::shared_ptr<QuadTree<T>>, 8>& temp) {
			//Pushing the Children to a temporary container
			for (uint8_t j = 0; j < NUMBER_OF_CHILDREN; ++j)
			{
				if (temp[j])
				{
					//If the pointer isn't null, I am placing it on to the queue
					temp_queue.push_back(temp[j]);
				}
				else
				{
					//Else I tell the function, that We propably reached the leaf node
					return false;
				}

			}

			//Comes down here when every octant has been found and pushed to the queue
			return true;
		};

		//Contains the main working queue
		std::list<std::shared_ptr<QuadTree<T>>> Children;

		//Checking if the root contains the item
		if (!m_Item.empty())
		{
			//Adding an item if it fits the QuadTree, or items if they cross through trees
			if (area.contains(m_Position))
			{
				for (const auto& it : m_Item)
				{
					items.push_back((it));
				}
			}
		}

		//Placing the root Children to the queue
		for (int i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			if (m_Children[i])
				Children.push_back(m_Children[i]);
		}

		//Iterative breadth first search implementation for an QuadTree
		size_t CurrentDepth = m_Depth;

		while (CurrentDepth < m_MaxDepth)
		{
			//Setting current depth
			CurrentDepth++;

			//For building the next Children queue
			std::list<std::shared_ptr<QuadTree<T>>> lower_Children;

			typename std::list<std::shared_ptr<QuadTree<T>>>::iterator it;

			//Using a Lambda, that enables returning only from the 2 nested loops
			[&] {
				//Checking the list
				for (it = Children.begin(); it != Children.end(); ++it)
				{
					//Take the AABB of the octant
					Collisions::AABB comparing = (**it).aabb();

					std::array<glm::vec3, 2> test = comparing.bounding_region();

					//Compare with the given area
					if (comparing.intersects2 (area)) {

						//Adding an item if it fits the QuadTree, or items if they cross through trees
						std::list<T> temp = (**it).access_elements();

						if (area.contains(comparing))
						{
							for (const auto& it : temp)
							{
								items.push_back((it));
							}
						}
						else if (is_leaf_node() && comparing.intersects2(area))
						{
							for (const auto& it : temp)
							{
								items.push_back((it));
							}
						}

					}

					//If the tree reached leaf nodes, this fails
					if (!asign_children(lower_Children, (**it).access_Children()))
					{
						return;
					}
				}
			}();

			//Swaping the queues
			std::swap(Children, lower_Children);
		}
	}


	//Checks whether the tree contains a certain area
	template<typename T>
	bool QuadTree<T>::contains(Collisions::AABB& area)
	{
		return m_Position.contains(area);
	}


	template<typename T>
	void QuadTree<T>::erase_area(Collisions::AABB& area, std::list<T>& items)
	{
		//Checking the parent node for the items
		if (!m_Item.empty())
		{
			//Adding an item if it fits the QuadTree, or items if they cross through trees
			if (area.contains(m_Position))
			{
				//Pushing the found item into the list
				for (const auto& it : m_Item)
				{
					items.push_back((it));
				}

				m_Item.clear();
			}
			else if (is_leaf_node() && m_Position.intersects2(area))
			{
				//Pushing the found item into the list
				for (const auto& it : m_Item)
				{
					items.push_back((it));
				}

				m_Item.clear();
			}
		}

		//Checking the child nodes
		for (int i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			if (m_Children[i])
			{
				//Checking for overlapping
				if (m_ChildrenBounds[i].intersects2(area))
				{
					m_Children[i]->erase_area(area, items);
				}
			}
		}

	}


	template<typename T>
	std::list<T> QuadTree<T>::access_elements()
	{
		return m_Item;
	}


	/*////////////////////
	* /    Modifiers     /
	*/////////////////////


	template<typename T>
	Trees::Location<T> QuadTree<T>::insert(T object, Collisions::AABB area)
	{
		//Checking whether anything can be inserted
		if (!m_NodeReady)
		{
			return {};
		}

		//
		return recursive_insert(object, area);

	}


	template<typename T>
	void QuadTree<T>::clear()
	{
		//Enabling the user to write a top-down new tree, by removing the locking flags
		m_Item.clear();

		//Proceeding to the Children
		for (int i = 0; i < NUMBER_OF_CHILDREN; i++)
		{

			if (m_Children[i])
			{
				// Recursively cleaning
				m_Children[i]->clear();
			}
		}
	}


	/*////////////////////
	* /  Space altering  /
	*/////////////////////


	template<typename T>
	void QuadTree<T>::shift(size_t leaf_nodes, Coordinates::Directions direction, std::list<std::pair<T, Collisions::AABB>>& returned_data)
	{
		//TODO: To redesign this function for it to suit the chunking system better. To specify the template for the chunks

		//Storing the new coordinates for the tree
		std::array<glm::vec3, 2> bounding_box = m_Position.bounding_region();

		//Temporary items list
		std::list<std::pair<T, Collisions::AABB>> items;

		//Calculating the bounding box
		switch (direction)
		{
		case Coordinates::Directions::North:

			bounding_box[0].z -= leaf_nodes * m_LeafNodeSide;
			bounding_box[1].z -= leaf_nodes * m_LeafNodeSide;

			break;
		case Coordinates::Directions::South:

			bounding_box[0].z += leaf_nodes * m_LeafNodeSide;
			bounding_box[1].z += leaf_nodes * m_LeafNodeSide;

			break;
		case Coordinates::Directions::East:

			bounding_box[0].x += leaf_nodes * m_LeafNodeSide;
			bounding_box[1].x += leaf_nodes * m_LeafNodeSide;

			break;
		case Coordinates::Directions::West:

			bounding_box[0].x -= leaf_nodes * m_LeafNodeSide;
			bounding_box[1].x -= leaf_nodes * m_LeafNodeSide;

			break;
		}

		//Placing all of the contained items into a list of pairs(item + coordinates)
		collect_items(items);

		//Resizing the tree
		resize({ bounding_box[0], bounding_box[1] });

		//Iterator of the items list
		typename std::list<std::pair<T, Collisions::AABB>>::iterator it;

		//Bulk inserting the content of the tree
		for (it = items.begin(); it != items.end(); ++it)
		{
			//If the item cannot be inserted, it means that is has been discarded
			if (!insert((*it).first, (*it).second).items_container)
			{
				returned_data.push_back(*it);
			}

		}

		//Clearing the temporary items list
		items.clear();
	}


	/*
	* //////////////////////////////
	* /  Private member functions  /
	* //////////////////////////////
	*/


	template<typename T>
	void QuadTree<T>::calculate_bounding_box(Collisions::AABB& BoundingBox, Children Octant)
	{
		// Here is the center of the parent node QuadTree
		glm::vec3 center = m_Position.center();

		// And It's side length for all dimensions
		glm::vec3 dimensions = m_Position.dimensions();

		// Variables for storing the min and max valus
		glm::vec3 minimum;
		glm::vec3 maximum;

		// Calculating the 8 different boudning boxes for the Children
		if (Octant == Children::O1)
		{
			// The minimum coordinate of the bounding box
			minimum = glm::vec3(center.x - 0.5 * dimensions.x, center.y, center.z);

			// The maximum coordinate of the bounding box
			maximum = glm::vec3(center.x, center.y + 0.5 * dimensions.y, center.z - 0.5 * dimensions.z);

			// Updating the given bounding box
			BoundingBox.update_position(minimum, maximum);

			return;
		}
		else if (Octant == Children::O2)
		{
			// The minimum coordinate of the bounding box
			minimum = center;

			// The maximum coordinate of the bounding box
			maximum = glm::vec3(center.x + 0.5 * dimensions.x, center.y + 0.5 * dimensions.y, center.z - 0.5 * dimensions.z);

			// Updating the given bounding box
			BoundingBox.update_position(minimum, maximum);

			return;
		}
		else if (Octant == Children::O3)
		{
			// The minimum coordinate of the bounding box
			minimum = glm::vec3(center.x - 0.5 * dimensions.x, center.y, center.z + 0.5 * dimensions.z);

			// The maximum coordinate of the bounding box
			maximum = glm::vec3(center.x, center.y + 0.5 * dimensions.y, center.z);

			// Updating the given bounding box
			BoundingBox.update_position(minimum, maximum);

			return;
		}
		else if (Octant == Children::O4)
		{
			// The minimum coordinate of the bounding box
			minimum = glm::vec3(center.x, center.y, center.z + 0.5 * dimensions.z);

			// The maximum coordinate of the bounding box
			maximum = glm::vec3(center.x + 0.5 * dimensions.x, center.y + 0.5 * dimensions.y, center.z);

			// Updating the given bounding box
			BoundingBox.update_position(minimum, maximum);

			return;
		}

	}


	template<typename T>
	bool QuadTree<T>::is_leaf_node(void)
	{
		return m_IsLeaf;
	}


	template<typename T>
	std::array<std::shared_ptr<QuadTree<T>>, 4>& QuadTree<T>::access_children()
	{
		return m_Children;
	}

	template<typename T>
	void QuadTree<T>::collect_items(std::list<std::pair<T, Collisions::AABB>>& items)
	{
		if (!m_Item.empty())
		{
			for (const auto& it : m_Item)
			{
				items.push_back({ (it), m_Position });
			}
		}
		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; ++i)
		{
			if (m_Children[i])
				m_Children[i]->collect_items(items);
		}
	}


	template<typename T>
	void QuadTree<T>::recursive_subdivide(void)
	{

		// If is a leaf node or the maximum depth has beed aproached
		if (!is_leaf_node())
		{
			return;
		}
		else if (!(m_Depth <= m_MaxDepth))
		{
			return;
		}

		//Getting the current bb dimensions
		glm::vec3 dimensions = m_Position.dimensions();

		//Safety checking whether the dimensions aren't smaller than the minimum value
		for (uint8_t i = 0; i < 3; i++)
		{
			if (dimensions[i] < MINIMUM_DIMENSION) return;
		}

		/*
		* Creating the coordinates of the Children, I am using a clever little technique,
		* by shifting bit-wise a number one, I achieve the needed enum class members values,
		* that allows to correctly recognize Children based on their hex code.
		*/

		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			calculate_bounding_box(m_ChildrenBounds[i], static_cast<Children>(1 << i));
		}

		//If it came down here It can't be a leaf node
		m_IsLeaf = false;

		//Creating the Children QuadTrees
		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			m_Children[i] = std::make_shared<QuadTree<T>>(m_ChildrenBounds[i], m_MaxDepth, m_MinimumDimensions, m_Depth + 1);
		}

	}


	template<typename T>
	void QuadTree<T>::recursive_dfs(Collisions::AABB& area, std::list<T>& items)
	{
		//Checking the parent node for the items
		if (!m_Item.empty())
		{
			//Adding an item if it fits the QuadTree, or items if they cross through trees
			if (m_Position.contains(area))
			{
				for (const auto& it : m_Item)
				{
					items.push_back((it));
				}
			}
			else if (is_leaf_node() && m_Position.intersects2(area))
			{
				for (const auto& it : m_Item)
				{
					items.push_back((it));
				}
			}
		}

		//Checking the child nodes
		for (int i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			if (m_Children[i])
			{
				//Checking for overlapping
				if (m_ChildrenBounds[i].intersects2(area))
				{
					m_Children[i]->recursive_dfs(area, items);
				}
			}
		}

	}

	template<typename T>
	Trees::Location<T> QuadTree<T>::recursive_insert(T object, Collisions::AABB area)
	{
		//Checking whether the children can contain the item
		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; i++)
		{
			if (m_ChildrenBounds[i].contains(area))
			{
				// Within the depth limit?
				if (m_Depth <= m_MaxDepth)
				{
					//If yes, does the child exist?
					if (!m_Children[i])
					{
						//If no, create that child
						m_Children[i] = std::make_shared<QuadTree<T>>(m_ChildrenBounds[i], m_Depth + 1, m_MinimumDimensions);
					}
					//If yes, proceed to the insertion
					return m_Children[i]->recursive_insert(object, area);
				}

			}

		}

		//Inserting an item
		if (m_Item.empty())
		{
			if (m_Position.contains(area))
			{
				//Inserting the object to the list 
				m_Item.push_back(object);

				//Returning the Dependencies::Tree::Location struct
				return { &m_Item, std::prev(m_Item.end()), area };
			}
			else
			{
				//Returning empty struct
				return {};
			}
		}
		else
		{
			//Returning empty struct
			return {};
		}

	}


	template<typename T>
	void QuadTree<T>::recursive_resize(Collisions::AABB& area)
	{
		//Temporary storage of the bounds
		std::array<glm::vec3, 2> bounds = area.bounding_region();

		//Updating the coordinates
		m_Position.update_position(bounds[0], bounds[1]);

		//Clearing the content
		m_Item.clear();

		// Calcaulating the potential children coordinates
		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; ++i)
		{
			calculate_bounding_box(m_ChildrenBounds[i], (Children)(1 << i));
		}

		for (uint8_t i = 0; i < NUMBER_OF_CHILDREN; ++i)
		{
			//Updating the Children with the bb data pulled from pre-calculated octantbounds
			if (m_Children[i])
				m_Children[i]->recursive_resize(m_ChildrenBounds[i]);
		}

	}

}
	


