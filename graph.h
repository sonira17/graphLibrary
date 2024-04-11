# include <vector>
# include <unordered_map>
# include <list>
# include <iostream>
# include <queue>
# include <stack>
# include <algorithm>
# include <limits.h>
# include <set>
using namespace std;
template <typename T>
class Graph{
   public:
  unordered_map<T, list<T>> adjList;
   void addEdge(int u,int v,bool direction){
    if(direction==1){
        adjList[u].push_back(v);
    }
    else{
         adjList[u].push_back(v);
         adjList[v].push_back(u);
    }
   }
   void printAdjacencyList(){
      for(auto i:adjList){
        cout<<i.first<<"- {";
       for(auto j:i.second){
        cout<<j<<",";
       }
       cout<<" }\n";
      }
   }
   unordered_map<int,list<pair<int,int>>> weighted;
   void weightedGraph(int u,int v,int w,bool direction){
       if(direction==1){
        weighted[u].push_back({v,w});
       }
       else{
         weighted[u].push_back({v,w});
         weighted[v].push_back({u,w});
       }
   }
   void printWeighted(){
    for(auto i:weighted){
        cout<<i.first<<"=[";
        for(auto j:i.second){
            cout<<","<<j.first<<","<<j.second;
        }
        cout<<"] "<<endl;
    }
   }
   vector<vector<int>> createAdjacencyMatrix(vector<pair<int,int>> &edgeList){
    int n=edgeList.size();
    vector<vector<int>> adjMatrix(n,vector<int>(n,0));
    for(auto i: edgeList){
        int u=i.first;
        int v=i.second;
        adjMatrix[u][v]=1;
    }
    return adjMatrix;
}

 unordered_map<T,bool> visited;
        void BfsTraversal(T src){
            // visited list
            queue<T> q;
            // initial state maintain karna h
            q.push(src);
            visited[src]=true;
            while(!q.empty()){
            T frontNode=q.front();
            q.pop();
            cout<<frontNode<<" ";
            for(auto nbr:adjList[frontNode]){
                    if(visited[nbr]!=true){
                        q.push(nbr);
                        visited[nbr]=true;
                    }
                }
            }
        }
        void callBfs(T src, T dst){
            for( T i=src;i<=dst;i++){
                if(visited[i]!=true){
                BfsTraversal(i);
                }
            }
        }

        void DfsTraversal(T src,unordered_map<T,bool> &vis){
            if(vis[src]==false){
            vis[src]=true;
            cout<<src<<" ";
            }
            for(auto i:adjList[src]){
               
                if(vis[i]!=true){
                    DfsTraversal(i,vis);                  
                }
            }
        }
        void callDfs(T src,T dst){
            unordered_map<T,bool> vis;
            for(T i=src;i<=dst;i++){
               if(vis[i]==false){
                   DfsTraversal(i,vis);
          }
              
        }
        }
        unordered_map<T,bool> v;
        bool cycleUndirectedGraph(T src){
          
           unordered_map<T,T> parent;
           queue<T> q;
           q.push(src);
           v[src]=true;
           parent[src]=-1;
           while(!q.empty()){
            T front=q.front();
            q.pop();
           for(auto nbr:adjList[front]){
            if(v[nbr]!=true){
                q.push(nbr);
                v[nbr]=true;
                parent[nbr]=front;
            }
            else if(v[nbr]==true&&nbr!=parent[front]){
                return true;
            }
           }
           }
           return false;
        }
        bool checkCycleBFSUndirected(T src,T dest){
            unordered_map<T,bool> v;
            bool ans=false;
            for(T i=src;i<=dest;i++){
             if(v[i]==false){
               ans= cycleUndirectedGraph(i);
             }
            }
           if (ans){
            return true;
           }
           else{
            return false;
           }
            }
      
      bool checkCycleUsingDfsUndirected(T src,unordered_map<T,bool>& visited,int parent){
        visited[src]=true;
        for(auto nbr:adjList[src]){
            if(parent==nbr){
                continue;
            }
            if(visited[nbr]==false){
                bool ans=checkCycleUsingDfsUndirected(nbr,visited,src);

                if(ans==true){
                    return true;
                }
            }
            else if(visited[nbr]==true) {
                return true;
            }
        }
        return false;
      }


    bool callCycleUndirectedDFS(T src,T dest){
        unordered_map<T,bool >visi;
        bool ans=false;
        for(T i=src;i<=dest;i++){
            int parent=-1;
            if(visi[i]==false){
            ans=checkCycleUsingDfsUndirected(i,visi,parent);
            }
        }
        if(ans){
            return true;
        }
        else{
            return false;
        }
    }


    bool checkCycleDirectedDfs(T src,unordered_map<T,bool> & vis,unordered_map<T,bool>& track){
        vis[src]=true;
        track[src]=true;
        for(auto nbr:adjList[src]){
             if(track[nbr]==true&&vis[nbr]==true){
                return true;
             }
             else if(vis[nbr]==false){
               bool ans= checkCycleDirectedDfs(nbr,vis,track);
               if(ans){
                return true;
               }
             }
        }
        track[src]=false;
        return false;
    }

    bool cycleUsingDfsDirected(T src,T dest){
        unordered_map<T,bool> visi;
        unordered_map<T,bool> track;
        bool ans=false;
        for(T i=src;i<=dest;i++){
            if(visi[i]==false){
                ans=checkCycleDirectedDfs(i,visi,track);
            }
        }
        return ans;
    }

   int connectedComponents(T src,T dst){
      int count=0;
      unordered_map<T,bool> vis;
      for(T i=src;i<=dst;i++){
        if(vis[i]==false){
         count++;
         DfsTraversal(i,vis);
        }
        
      }
      return count;
    }
   void topologicalUsingDfs(T src, unordered_map<T,bool> &vis,stack<int> &st){
        vis[src]=true;
        for(auto i: adjList[src]){
            if(vis[i]!=true){
                topologicalUsingDfs(i,vis,st);
              
            }
        }
        
         st.push(src);
    } 

    void calltopologicalUsingDfs(T src,T dest){
        unordered_map<T,bool> vis;
        stack<T> st;
        for(T i=src;i<=dest;i++){
            if(vis[i]==false){
             topologicalUsingDfs(i,vis,st);
            }
            
        }
        while(!st.empty()){
            cout<<st.top()<<" ";
            st.pop();
        }
    }

    void topological_sortBfs(int n){
        queue<T> q;
       unordered_map<T,int> indegree;
        // calculating indegree
        for(auto i: adjList){
            for(auto j:i.second){
                indegree[j]++;
            }
        }
        for(auto i=0;i<n;i++){
            if(indegree[i]==0){
                ans.push_back(i);
                q.push(i);
            }
        }
        while(!q.empty()){
          T front=q.front();
          cout<<front<<" ";
          q.pop();
          for(auto nbr:adjList[front]){
             indegree[nbr]--;
             if(indegree[nbr]==0){
                ans.push_back(nbr);
                q.push(nbr);
             }
          }
        }
    }
     vector<T> ans;
    checkCycleInDirectedGraphBfs(int n){
       
        topological_sortBfs(n);
        if(ans.size()==n){
            cout<<"cycle is not present"<<endl;
        }
        else{
            cout<<"cycle is present"<<endl;
        }
    }

     vector<T> shortestPathUsingBfs(T src, T dst){
        queue<T> q;
        unordered_map<T,bool> vis;
        unordered_map<T,T> parent;
        q.push(src);
        vis[src]=true;
        parent[src]=-1;
        while(!q.empty()){
            T front=q.front();
            q.pop();
            for(auto nbr:adjList[front]){
                if(vis[nbr]!=true){
                    q.push(nbr);
                    vis[nbr]=true;
                    parent[nbr]=front;
                }
            }
        }
        vector<T> ans;
        while(dst!=-1){
            ans.push_back(dst);
            dst=parent[dst];
        }
        reverse(ans.begin(),ans.end());
        return ans;
     }

     void topOrdereDfs(T src,unordered_map<T,bool> &vis, stack<T>& st){
        vis[src]=true;
        for(auto nbr: weighted[src]){
            T nbrNode=nbr.first;
            if(vis[nbrNode]!=true){
                topOrdereDfs(nbrNode,vis,st);
            }
        }
        st.push(src);        
     }

     void shortestPathDfs(int n,T dst){
       stack<T> st;
       unordered_map<T,bool> vis;
       topOrdereDfs(0,vis,st);
       vector<int> dis(n,INT_MAX);
        T src=st.top();
         st.pop();
         dis[src]=0;
          for(auto nbr: weighted[src]){
         T nbrNode=nbr.first;
          int wt=nbr.second;
         int d=dis[src]+wt;
         if(d<dis[nbrNode]){
            dis[nbrNode]=d;
         }
       }
       
       while(!st.empty()){
        T src=st.top();
        st.pop();
         for(auto nbr:weighted[src]){
        T nbrNode=nbr.first;
        int wt=nbr.second;
         int d=dis[src]+wt;
         if(d<dis[nbrNode]){
            dis[nbrNode]=d;
         }
       }
       }
       for(auto i:dis){
        cout<<i<<" ";
       }
     }

     void dijkstra(T src,T dest,int n){
        unordered_map<T,T> parent;
        set<pair<int,T>> st;
        vector<int> dis(n+1,INT_MAX);
        // initial state maintian karna h
        st.insert({0,src});
        parent[src]=-1;
        dis[src]=0;

        while(!st.empty()){
            auto topElement=st.begin();
            pair<int,T> topPair=*topElement;
            int topDis=topPair.first;
            int topNode=topPair.second;
            st.erase(st.begin());
            for(pair<int,int> nbr:weighted[topNode]){
                 int Node=nbr.first;
                 int Dis=nbr.second;
                 if(topDis+Dis<dis[Node]){
                    // hume update krna h distance ko
                   auto prevEntry=st.find({dis[Node],Node});
                    if(prevEntry!=st.end()){
                       st.erase(prevEntry);
                    }                   
                    // nayi wali entry ko create krna h
                    dis[Node]=topDis+Dis;
                    st.insert({dis[Node],Node});
                    parent[Node]=topNode;
                 }
            }
        }
    cout<<"Shortest distance between "<<src<<" and "<<dest<<" is "<<dis[dest]<<endl;
      vector<int> ans;
       while(dest!=-1){
         ans.push_back(dest);
         dest=parent[dest];
       }
       reverse(ans.begin(),ans.end());
       cout<<"Shortest path using Dijkstra :"<<endl;
       for(int i=0;i<ans.size();i++){
        cout<<ans[i]<<" ";
       }
     }

    vector<int> bellmanFord(int n,T src){
        vector<int> dis(n,INT_MAX);
        dis[src-'A'] = 0;
        for(int k=1;k<n;k++){
            for(auto i:weighted){
            T node=i.first;
                for(auto j:i.second){
                   T nbr=j.first;
                    int wt=j.second;
                    if(dis[node-'A']!=INT_MAX&&dis[node-'A']+wt<dis[nbr-'A']){
                        dis[nbr-'A']=dis[node-'A']+wt;
                        
                    }
                }
            }
        }
        return dis;
    }

    bool checkNegativeCycle(int n, T src){
         
        vector<int> dis(n,INT_MAX);
        dis[src-'A'] = 0;
        for(int k=1;k<n;k++){
            for(auto i:weighted){
            T node=i.first;
                for(auto j:i.second){
                   T nbr=j.first;
                    int wt=j.second;
                    if(dis[node-'A']!=INT_MAX&&dis[node-'A']+wt<dis[nbr-'A']){
                        dis[nbr-'A']=dis[node-'A']+wt;
                        
                    }
                }
            }
        }
         bool flag=false;
          for(auto i:weighted){
            T node=i.first;
                for(auto j:i.second){
                   T nbr=j.first;
                    int wt=j.second;
                    if(dis[node-'A']!=INT_MAX&&dis[node-'A']+wt<dis[nbr-'A']){
                        dis[nbr-'A']=dis[node-'A']+wt;
                        flag=true;
                        break;
                    }
                }
            }
            return flag;
    }

    vector<vector<int>> fylodWarshall(int n){
        vector<vector<int>> dis(n,vector<int>(n,1e9));
        // initail state maintain krna h
        for(int i=0;i<n;i++){
            dis[i][i]=0;
        }
        for(auto i:weighted){
            for(auto j:i.second){
                int u=i.first;
                int v=j.first;
                int wt=j.second;
                dis[u][v]=wt;
            }
        }

         for(int helper=0;helper<n;helper++){
            for(int src=0;src<n;src++){
                for(int dst=0;dst<n;dst++){
                   dis[src][dst]=min(dis[src][dst],(dis[src][helper]+dis[helper][dst]));
                }
            }
         }
       return dis;
     
    }
    void kosarajuOrdering(T src,unordered_map<T,bool> &vis,stack<T> &st){
     vis[src]=true;
     for(auto nbr:adjList[src]){
        if(vis[nbr]==false){
            kosarajuOrdering(nbr,vis,st);
        }
     } 
    
      st.push(src);
      //cout<<st.top();
    }
    unordered_map<T,list<T>> revList;
    void kosarajuRevrse(){
        for(auto u:adjList){
            for(auto v:u.second){
                revList[v].push_back(u.first);
            }
        }
    }

    void DfsTraversal1(T src,unordered_map<T,bool> &vi){
        vi[src]=true;
        for(auto nbr:revList[src]){
            if(vi[nbr]==false){
                DfsTraversal1(nbr,vi);
            }
        }
    }
    int kosarajCount(stack<T> st){
        int count=0;
        unordered_map<T,bool> vis1;
        while(!st.empty()){
            T temp=st.top();
            st.pop();
            if(vis1[temp]==false){
                DfsTraversal1(temp,vis1);
                count++;
            }
        }
        return count;
    }
     int findMinimum(vector<int> key,vector<bool> mst){
        int temp=INT_MAX;
        int index=-1;
        for(int i=0;i<key.size();i++){
            if(key[i]<temp&&mst[i]==false){
                temp=key[i];
                index=i;
            }
        }
      
        return index;
     }   
   void primsAlgo(int src,int n){
    vector<int> key(n,INT_MAX);
    vector<bool> mst(n,false);
    vector<int> parent(n,-1);
    // initail state maintain karna h
    key[src]=0;
     while(true){
        int u=findMinimum(key,mst);
        if(u==-1) break;
        mst[u]=true;
        for(auto adj:weighted[u]){
            int v=adj.first;
            int w=adj.second;
            if(mst[v]==false&&w<key[v]){
                key[v]=w;
                parent[v]=u;
                
            }
        }
     }
     for(int i=0;i<parent.size();i++){
        cout<<parent[i]<<" ";
     }
     int sum=0;
     for(int u=0;u<parent.size();u++){
        if(parent[u]==-1) continue;
        for(auto nbr:weighted[u]){
            int v=nbr.first;
            int w=nbr.second;
            if(v==parent[u]){
                sum=sum+w;
            }
        }
     }
     cout<<"Sum"<<sum;
   }
   static bool myComp(vector<int>& a,vector<int>& b){
    return a[2]<b[2];
   }
   int findParent(vector<int>& parent,int node){
    if(node==parent[node]) return node;
    else {
        parent[node]=findParent(parent,parent[node]);
    }
    return parent[node];
   }

   void unionSet(int u,int v,vector<int> &parent,vector<int> &rank){
       u=findParent(parent,u);
       v=findParent(parent,v);
       if(rank[u]<rank[v]){
        parent[u]=v;
        rank[v]++;
       }
       else{
        parent[v]=u;
        rank[u]++;
       }
   }
   int kruskalsAlgorithm(int n){
    vector<int> parent(n);
    for(int i=0;i<n;i++){
        parent[i]=i;
    }
    vector<int> rank(n,0);
    vector<vector<int>> edges;
    // creating the linear data structure
    for(auto i:weighted){
        int u=i.first;
        for(auto j:i.second){
           int v=j.first;
           int wt=j.second;
           edges.push_back({u,v,wt});
         
        }
    }
    sort(edges.begin(), edges.end(),myComp);

  
     int ans=0;
     for(auto edge:edges){
        int u=edge[0];
        int v=edge[1];
        int wt=edge[2];
        cout<<" u="<<u<<"v="<<v<<"wt="<<wt<<endl;
        u=findParent(parent,u);
        v=findParent(parent,v);
        if(u!=v){
            unionSet(u,v,parent,rank);
            ans+=wt;
          cout<<"ans"<<ans;
        }
     }
     return ans;
     
   }
};



