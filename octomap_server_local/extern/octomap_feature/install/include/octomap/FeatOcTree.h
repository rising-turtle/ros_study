/*
Author: SHEN Hao
Data:	2014.1.7

info: extend the Octree node to store SURF feature 
*/
#ifndef OCTOMAP_FEATURE_OCTREE_H
#define OCTOMAP_FEATURE_OCTREE_H

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#define MAX_FEAT_NUM	10
#define FEAT_SIZE		64
namespace octomap {
	// node definition
	class FeatOcTreeNode : public OcTreeNode {   
	public:
		//color info
		class Color {
		public:
			Color() : r(255), g(255), b(255) {}
			Color(unsigned char _r, unsigned char _g, unsigned char _b) 
				: r(_r), g(_g), b(_b) {}
			inline bool operator== (const Color &other) const {
				return (r==other.r && g==other.g && b==other.b);
			}
			inline bool operator!= (const Color &other) const {
				return (r!=other.r || g!=other.g || b!=other.b);
			}
			unsigned char r, g, b;
		};

		//////////////////////////////////////////////////////////////////////////
		// add feature class
		//feature info
		class Feature{
		public:
			Feature() : feat_cnt(0) {}
			~Feature() {
				feat_cnt--;
				while(feat_cnt>=0)
				{
					delete[] feat_desc[feat_cnt];
					feat_cnt--;
				}
			}
			int feat_cnt;					//stored feature num
			float* feat_desc[MAX_FEAT_NUM];	//feature descriptors, need to allocate memory when insert descriptor
			
		};
		//////////////////////////////////////////////////////////////////////////

	public:

		//only performed once, when the function updateNode was called multi time 
		FeatOcTreeNode() : OcTreeNode() {
			feats.feat_cnt = 0;
			weight = 0;
		}

			FeatOcTreeNode(const FeatOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color) {}

		bool operator==(const FeatOcTreeNode& rhs) const{
			return (rhs.value == value && rhs.color == color);
		}

		// children
		inline FeatOcTreeNode* getChild(unsigned int i) {
			return static_cast<FeatOcTreeNode*> (OcTreeNode::getChild(i));
		}
		inline const FeatOcTreeNode* getChild(unsigned int i) const {
			return static_cast<const FeatOcTreeNode*> (OcTreeNode::getChild(i));
		}

		bool createChild(unsigned int i) {
			if (children == NULL) allocChildren();
			children[i] = new FeatOcTreeNode();
			return true;
		}

		bool pruneNode();
		void expandNode();

		inline Color getColor() const { return color; }
		inline void  setColor(Color c) {this->color = c; }
		inline void  setColor(unsigned char r, unsigned char g, unsigned char b) {
			this->color = Color(r,g,b); 
		}

		Color& getColor() { 
			return color; }

		// has any color been integrated? (pure white is very unlikely...)
		inline bool isColorSet() const { 
			return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
		}

		void updateColorChildren();


		FeatOcTreeNode::Color getAverageChildColor() const;

		// file I/O
		std::istream& readValue (std::istream &s);
		std::ostream& writeValue(std::ostream &s) const;

		//////////////////////////////////////////////////////////////////////////
		//added function
		inline void  addFeat(const float* desc, int nDim) {
			if (nDim!=FEAT_SIZE)
			{
				return;
			}
			// when feature num reaches the max limitation, delete one feature randomly
			if (feats.feat_cnt>=MAX_FEAT_NUM)
			{
				time_t t;
				srand((unsigned) time(&t));				
				int index_ = rand() % MAX_FEAT_NUM;
				for (int i=index_; i<MAX_FEAT_NUM-1; i++)
				{
					memcpy(feats.feat_desc[i], feats.feat_desc[i+1], FEAT_SIZE*sizeof(float));
				}
				memcpy(feats.feat_desc[MAX_FEAT_NUM-1], desc, FEAT_SIZE*sizeof(float));
				feats.feat_cnt = MAX_FEAT_NUM;
			}
			else
			{
				this->feats.feat_desc[this->feats.feat_cnt] = new float[FEAT_SIZE];
				memcpy(this->feats.feat_desc[this->feats.feat_cnt], desc, nDim*sizeof(float));
				this->feats.feat_cnt++; 
			}			
		}

		//copy f to feats
		inline void  addFeat(Feature f) {
			this->feats.feat_cnt = f.feat_cnt; 
			f.feat_cnt = 0;
			while(f.feat_cnt<this->feats.feat_cnt)
			{
				this->feats.feat_desc[f.feat_cnt] = new float[FEAT_SIZE];
				memcpy(this->feats.feat_desc[f.feat_cnt], f.feat_desc[f.feat_cnt], FEAT_SIZE*sizeof(float));
				f.feat_cnt++;
			}

		}
		inline void getFeat(Feature& feat_dst) { 
			feat_dst.feat_cnt = feats.feat_cnt;
			feats.feat_cnt = 0; 
			while(this->feats.feat_cnt<feat_dst.feat_cnt)
			{
				feat_dst.feat_desc[this->feats.feat_cnt] = new float[FEAT_SIZE];
				memcpy(feat_dst.feat_desc[this->feats.feat_cnt], this->feats.feat_desc[this->feats.feat_cnt], FEAT_SIZE*sizeof(float));
				this->feats.feat_cnt++;
			}
		}

		inline int getFeatNum() const {
			return feats.feat_cnt; }

		void deleteFeats();

	protected:
		Color		color;
		Feature		feats;
		double		weight;


	};

	//////////////////////////////////////////////////////////////////////////
	// tree definition
	class FeatOcTree : public OccupancyOcTreeBase <FeatOcTreeNode> {
	public:
		/// Default constructor, sets resolution of leafs
		FeatOcTree(double resolution) : OccupancyOcTreeBase<FeatOcTreeNode>(resolution) {};  

		/// virtual constructor: creates a new object of same type
		/// (Covariant return type requires an up-to-date compiler)
		FeatOcTree* create() const {return new FeatOcTree(resolution); }

		std::string getTreeType() const {return "FeatOcTree";}

		// set node color at given key or coordinate. Replaces previous color.
		FeatOcTreeNode* setNodeColor(const OcTreeKey& key, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b);

		FeatOcTreeNode* setNodeColor(const float& x, const float& y, 
			const float& z, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
				return setNodeColor(key,r,g,b);
		}

		// set node feat at given key or coordinate. .
		FeatOcTreeNode* addNodeFeat(const OcTreeKey& key, const float* desc, const int dim);

		FeatOcTreeNode* addNodeFeat(const float& x, const float& y, 
			const float& z, const float* desc, const int dim) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
				return addNodeFeat(key, desc, dim);
		}

		// get node feat at given key or coordinate. .
		FeatOcTreeNode* getNodeFeat(const OcTreeKey& key, FeatOcTreeNode::Feature& feats);

		FeatOcTreeNode* getNodeFeat(const float& x, const float& y, 
			const float& z, FeatOcTreeNode::Feature& feats) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
				return getNodeFeat(key, feats);
		}

		// remove feature node
		void removeNodeFeat(const OcTreeKey& key);
		void removeNodeFeat(const float& x, const float& y, 
			const float& z) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return ;
				removeNodeFeat(key);
		}
		// integrate color measurement at given key or coordinate. Average with previous color
		FeatOcTreeNode* averageNodeColor(const OcTreeKey& key, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b);

		FeatOcTreeNode* averageNodeColor(const float& x, const float& y, 
			const float& z, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
				return averageNodeColor(key,r,g,b);
		}

		// integrate color measurement at given key or coordinate. Average with previous color
		FeatOcTreeNode* integrateNodeColor(const OcTreeKey& key, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b);

		FeatOcTreeNode* integrateNodeColor(const float& x, const float& y, 
			const float& z, const unsigned char& r, 
			const unsigned char& g, const unsigned char& b) {
				OcTreeKey key;
				if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
				return integrateNodeColor(key,r,g,b);
		}

		// update inner nodes, sets color to average child color
		void updateInnerOccupancy();

		// uses gnuplot to plot a RGB histogram in EPS format
		//void writeColorHistogram(std::string filename);

	protected:
		void updateInnerOccupancyRecurs(FeatOcTreeNode* node, unsigned int depth);

		/**
		* Static member object which ensures that this OcTree's prototype
		* ends up in the classIDMapping only once
		*/
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				FeatOcTree* tree = new FeatOcTree(0.1);
				AbstractOcTree::registerTreeType(tree);
			}
		};
		/// static member to ensure static initialization (only once)
		static StaticMemberInitializer featOcTreeMemberInit;

	};

	//! user friendly output in format (r g b)
	std::ostream& operator<<(std::ostream& out, FeatOcTreeNode::Color const& c);

}// end namespace


#endif
