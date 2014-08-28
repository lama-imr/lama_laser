/* Base class for learning jockeys
 */

#ifndef _LAMA_INTERFACES_LEARNING_JOCKEY_H_
#define _LAMA_INTERFACES_LEARNING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_interfaces/jockey.h>
#include <lama_interfaces/LearnAction.h>
#include <lama_interfaces/LearnGoal.h>
#include <lama_interfaces/LearnFeedback.h>

namespace lama
{
namespace interfaces
{

typedef actionlib::SimpleActionServer<lama_interfaces::LearnAction> LearnServer;

class LearningJockey : public Jockey
{
  public:

    LearningJockey(std::string name);
    ~LearningJockey();

    virtual void onStartLearn() = 0;
    virtual void onStopLearn() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

  protected:

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur (this is done in Jockey).
    LearnServer server_;
    lama_interfaces::LearnResult result_;
    lama_interfaces::LearnFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the attributes of current goal
    // are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_interfaces::LearnGoal goal_;

  private:

    void goalCallback();
    void preemptCallback();

    // Change the visibility to avoid double calls.
    using Jockey::initAction;
    using Jockey::interrupt;
    using Jockey::resume;
};

} // namespace interfaces
} // namespace lama

#endif // _LAMA_INTERFACES_LEARNING_JOCKEY_H_
