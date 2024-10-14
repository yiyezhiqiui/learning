#include "../include/tree.hpp"
int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");

    // Find all the XML files in a folder and register all of them.
    // We will use std::filesystem::directory_iterator
    std::string search_directory = "./src/";

    using std::filesystem::directory_iterator;
    for (auto const &entry : directory_iterator(search_directory))
    {
        if (entry.path().extension() == ".xml")
        {
            factory.registerBehaviorTreeFromFile(entry.path().string());
        }
    }
    // This, in our specific case, would be equivalent to
    // factory.registerBehaviorTreeFromFile("./main_tree.xml");
    // factory.registerBehaviorTreeFromFile("./subtree_A.xml");
    // factory.registerBehaviorTreeFromFile("./subtree_B.xml");

    // You can create the MainTree and the subtrees will be added automatically.
    std::cout << "----- MainTree tick ----" << std::endl;
    auto main_tree = factory.createTree("MainTree");
    main_tree.tickWhileRunning();

    // ... or you can create only one of the subtrees
    std::cout << "----- SubA tick ----" << std::endl;
    auto subA_tree = factory.createTree("SubTreeA");
    subA_tree.tickWhileRunning();

    // ... or you can create only one of the subtrees
    std::cout << "----- SubA tick ----" << std::endl;
    subA_tree.tickWhileRunning();

    return 0;
}
/* Expected output:

Registered BehaviorTrees:
 - MainTree
 - SubTreeA
 - SubTreeB
----- MainTree tick ----
Robot says: starting MainTree
Robot says: Executing Sub_A
Robot says: Executing Sub_B
----- SubA tick ----
Robot says: Executing Sub_A
*/