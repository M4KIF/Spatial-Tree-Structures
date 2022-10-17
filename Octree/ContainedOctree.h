//Dependencies
#include "Octree.h"


/*
* This container is meant to implement
* the Octree in conjunction with a list
* This way way, the
* Octree will not own any item, but only
* iterators to the items, It's cheaper* 
*/

namespace Trees {

	template<typename T>
	struct Location
	{
		typename std::list<T>* items_container = nullptr;
		typename std::list<T>::iterator items_iterator;
		typename Collisions::AABB aabb;
	};

}

namespace DataStructures {

	template<typename T>
	struct OctreeItem
	{
		//Item itself
		T item;

		//The location to the container inside Octree that holds the iterator to this exact element above
		Trees::Location<typename std::list<OctreeItem<T>>::iterator> item_position;
	};

	template<typename T>
	class ContainedOctree
	{

		using OctreeContainer = std::list<OctreeItem<T>>;

	protected:

		Octree<typename OctreeContainer::iterator> m_Root;
		OctreeContainer m_Items;

	public:

		/*
		* Initialisation
		*/

		ContainedOctree();
		ContainedOctree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions);
		ContainedOctree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions, std::list<T> Items);
		~ContainedOctree();

		/*
		* Dimensions & Position
		*/

		size_t min_dimensions();
		Collisions::AABB& aabb();
		void resize(Collisions::AABB area);

		/*////////
		* Capacity
		*/////////

		size_t size();
		size_t max_size();
		size_t depth();
		size_t max_depth();
		bool empty();

		/*
		* Element access
		*/

		//Iterators that enable using this container in a for loop(optional)
		typename std::list<T>::iterator begin();
		typename std::list<T>::iterator end();
		typename std::list<T>::iterator cbegin();
		typename std::list<T>::iterator cend();

		//Search functions
		void dfs(Collisions::AABB& area, std::list<typename OctreeContainer::iterator>& items);
		void bfs(Collisions::AABB& area, std::list<typename OctreeContainer::iterator>& items);
		bool contains(Collisions::AABB& area);

		//Others
		std::vector<T> items();

		/*
		* Modifiers
		*/

		bool insert(T object, Collisions::AABB area);
		bool remove(typename OctreeContainer::iterator& item);
		void clear();

		/*
		* Space altering
		*/

		void shift(size_t leaf_nodes, Coordinates::Directions direction, std::list<std::pair<T, Collisions::AABB>>& returned_data); //TODO

	};


	/*
	* ///////////////////////
	* /		Definitions     /
	* ///////////////////////
	*/


	template<typename T>
	ContainedOctree<T>::ContainedOctree() :
		m_Root()
	{}
	

	template<typename T>
	ContainedOctree<T>::ContainedOctree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions) :
		m_Root(BoundingBox, MaxDepth, MinimumDimensions)
	{}


	template<typename T>
	ContainedOctree<T>::ContainedOctree(Collisions::AABB BoundingBox, size_t MaxDepth, size_t MinimumDimensions, std::list<T> Items) :
		m_Root(BoundingBox, MaxDepth, MinimumDimensions)
	{
		// TODO: make a bulk insertion algorithm
	}

	template<typename T>
	ContainedOctree<T>::~ContainedOctree()
	{
		clear();
	}


	/*////////////////////
	* /     Capacity     /
	*/////////////////////

	template<typename T>
	Collisions::AABB& ContainedOctree<T>::aabb()
	{
		return m_Root.aabb();
	}


	template<typename T>
	size_t ContainedOctree<T>::size()
	{
		return (size_t)m_Items.size();
	}


	template<typename T>
	size_t ContainedOctree<T>::max_size()
	{
		return m_Root.max_size();
	}


	template<typename T>
	size_t ContainedOctree<T>::min_dimensions()
	{
		return m_Root.min_dimensions();
	}


	template<typename T>
	size_t ContainedOctree<T>::max_depth()
	{
		return m_Root.max_depth();
	}


	template<typename T>
	void ContainedOctree<T>::resize(Collisions::AABB area)
	{
		//Cleaning the tree of the iterators
		m_Root.resize(area);

		//Cleaning the list
		m_Items.clear();
	}


	template<typename T>
	bool ContainedOctree<T>::empty()
	{
		return m_Items.empty();
	}


	/*////////////////////
	* / Element Access   /
	*/////////////////////


	template<typename T>
	typename std::list<T>::iterator ContainedOctree<T>::begin()
	{
		return m_Items.begin();
	}


	template<typename T>
	typename std::list<T>::iterator ContainedOctree<T>::end()
	{
		return m_Items.end();
	}


	template<typename T>
	typename std::list<T>::iterator ContainedOctree<T>::cbegin()
	{
		return m_Items.cbegin();
	}


	template<typename T>
	typename std::list<T>::iterator ContainedOctree<T>::cend()
	{
		return m_Items.cend();
	}


	template<typename T>
	void ContainedOctree<T>::dfs(Collisions::AABB& area, std::list<typename OctreeContainer::iterator>& items)
	{
		m_Root.dfs(area, items);
	}


	template<typename T>
	void ContainedOctree<T>::bfs(Collisions::AABB& area, std::list<typename OctreeContainer::iterator>& items)
	{
		m_Root.bfs(area, items);
	}


	template<typename T>
	bool ContainedOctree<T>::contains(Collisions::AABB& area)
	{
		return m_Root.contains(area);
	}


	template<typename T>
	std::vector<T> ContainedOctree<T>::items()
	{
		//Stores the found data
		std::vector<T> Items;

		//Pushing available items to the vector
		for (const auto& it : m_Items)
		{
			Items.push_back(it->item);
		}
	}


	/*////////////////////
	* /    Modifiers     /
	*/////////////////////


	template<typename T>
	bool ContainedOctree<T>::insert(T object, Collisions::AABB area)
	{
		//Temporary storage for the Dependencies::Tree::Location object
		OctreeItem<T> temp;

		//Inserting the item to the structure
		temp.item = object;

		//Pushing the structure up the list
		m_Items.push_back(temp);

		//Filling the remaining data, that We get from the Octree insertion
		m_Items.back().item_position = m_Root.insert(std::prev(m_Items.end()), area);

		//Depending on the outcome of the insertion gives the result
		if (m_Items.back().item_position.items_container)
		{
			return true;
		}
		else
			return false;
	}


	template<typename T>
	bool ContainedOctree<T>::remove(typename OctreeContainer::iterator& item)
	{
		/*Basicly, acceses the iterator, finds the container in the accessed structure,
		finds the iterator in the structure, and demands the container to erase the given iterator from its content*/
		item->item_position.items_container->erase(item->item_position.items_iterator);

		//Deletes the original item from the list
		m_Items.erase(item);

		//
		return false;
	}


	template<typename T>
	void ContainedOctree<T>::clear()
	{
		//And the whole Octree
		m_Root.clear();

		//Clears the list of items

		//TODO: CLEAR THE NEW STRUCTS BEFORE DELETING

		m_Items.clear();
	}


	/*////////////////////
	* /  Space altering  /
	*/////////////////////

	template<typename T>
	void ContainedOctree<T>::shift(size_t leaf_nodes, Coordinates::Directions direction, std::list<std::pair<T, Collisions::AABB>>& returned_data)
	{
		//Storing the new coordinates for the tree
		std::array<glm::vec3, 2> bounding_box = m_Root.aabb().bounding_region();

		//Getting the side length for the calculations
		size_t leaf_side_length = m_Root.leaf_node_side_length();

		//Calculating the bounding box
		switch (direction)
		{
		case Coordinates::Directions::North:

			bounding_box[0].z -= leaf_nodes * leaf_side_length;
			bounding_box[1].z -= leaf_nodes * leaf_side_length;

			break;
		case Coordinates::Directions::South:

			bounding_box[0].z += leaf_nodes * leaf_side_length;
			bounding_box[1].z += leaf_nodes * leaf_side_length;

			break;
		case Coordinates::Directions::East:

			bounding_box[0].x += leaf_nodes * leaf_side_length;
			bounding_box[1].x += leaf_nodes * leaf_side_length;

			break;
		case Coordinates::Directions::West:

			bounding_box[0].x -= leaf_nodes * leaf_side_length;
			bounding_box[1].x -= leaf_nodes * leaf_side_length;

			break;
		}

		//Resizing the tree
		m_Root.resize({ bounding_box[0], bounding_box[1] });

		//Iterator of the items list
		typename std::list<OctreeItem<T>>::iterator it = m_Items.begin();

		//Bulk inserting the content of the tree
		while (it != m_Items.end())
		{

			//If the item cannot be inserted, it means that is has been discarded
			if (!(m_Root.insert(it, it->item_position.aabb)).items_container)
			{
				//Giving the info about the item that didn't fit
				returned_data.push_back({it->item, it->item_position.aabb});

				//Incrementing the iterator
				it++;

				//Deleting and proceeding further
				m_Items.erase(std::prev(it));
				continue;
			}
			
			//Incrementing the iterator
			++it;

		}

	}

}
