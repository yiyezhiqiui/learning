#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

int main()
{
    BT::BehaviorTreeFactory factory;

    auto tree = factory.createTreeFromFile("src/mytree.xml");

    // Helper function to print the tree.
    BT::printTreeRecursively(tree.rootNode());

    // The purpose of the observer is to save some statistics about the number of times
    // a certain node returns SUCCESS or FAILURE.
    // This is particularly useful to create unit tests and to check if
    // a certain set of transitions happened as expected
    BT::TreeObserver observer(tree);

    // Print the unique ID and the corresponding human readable path
    // Path is also expected to be unique.
    std::map<uint16_t, std::string> ordered_UID_to_path;
    for (const auto &[name, uid] : observer.pathToUID())
    {
        ordered_UID_to_path[uid] = name;
    }

    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        //std::cout << uid << " -> " << name << std::endl;
    }

    tree.tickWhileRunning();

    // You can access a specific statistic, using is full path or the UID
    const auto &last_action_stats = observer.getStatistics("last_action");
    assert(last_action_stats.transitions_count > 0);

    std::cout << "test----" << std::endl;
    // print all the statistics
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        const auto &stats = observer.getStatistics(uid);

        std::cout << "[" << name
                  << "] \tT/S/F:  " << stats.transitions_count
                  << "/" << stats.success_count
                  << "/" << stats.failure_count
                  << std::endl;
    }

    return 0;
}