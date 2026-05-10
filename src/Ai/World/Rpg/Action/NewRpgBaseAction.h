#ifndef _PLAYERBOT_NEWRPGBASEACTION_H
#define _PLAYERBOT_NEWRPGBASEACTION_H

#include "Duration.h"
#include "LastMovementValue.h"
#include "MovementActions.h"
#include "NewRpgInfo.h"
#include "NewRpgStrategy.h"
#include "Object.h"
#include "ObjectDefines.h"
#include "ObjectGuid.h"
#include "PlayerbotAI.h"
#include "QuestDef.h"
#include "TravelMgr.h"

struct POIInfo
{
    G3D::Vector2 pos;
    int32 objectiveIdx;
};

/// A base (composition) class for all new rpg actions
/// All functions that may be shared by multiple actions should be declared here
/// And we should make all actions composable instead of inheritable
class NewRpgBaseAction : public MovementAction
{
public:
    NewRpgBaseAction(PlayerbotAI* botAI, std::string name) : MovementAction(botAI, name) {}

protected:
    /* MOVEMENT RELATED */
    bool MoveFarTo(WorldPosition dest);
    bool MoveWorldObjectTo(ObjectGuid guid, float distance = INTERACTION_DISTANCE);
    bool MoveRandomNear(float moveStep = 50.0f, MovementPriority priority = MovementPriority::MOVEMENT_NORMAL, WorldObject* center = nullptr);
    bool ForceToWait(uint32 duration, MovementPriority priority = MovementPriority::MOVEMENT_NORMAL);
    bool TakeFlight(std::vector<uint32> const& taxiNodes, Creature* flightMaster);

    /* QUEST RELATED CHECK */
    ObjectGuid ChooseNpcOrGameObjectToInteract(bool questgiverOnly = false, float distanceLimit = 0.0f);
    bool HasQuestToAcceptOrReward(WorldObject* object);
    bool InteractWithNpcOrGameObjectForQuest(ObjectGuid guid);
    bool CanInteractWithQuestGiver(Object* questGiver);
    bool IsWithinInteractionDist(Object* object);
    uint32 BestRewardIndex(Quest const* quest);
    bool IsQuestWorthDoing(Quest const* quest);
    bool IsQuestCapableDoing(Quest const* quest);

    /* QUEST RELATED ACTION */
    bool SearchQuestGiverAndAcceptOrReward();
    bool AcceptQuest(Quest const* quest, ObjectGuid guid);
    bool TurnInQuest(Quest const* quest, ObjectGuid guid);
    bool OrganizeQuestLog();

    /* QUEST PROGRESSION HELPERS (at POI) */
    // Walk to a GO that drops a needed quest item. The loot strategy
    // opens and loots it once in range.
    bool TryLootQuestGO(ObjectGuid& pursuedGO, float searchRange = 60.0f);

    // Walk to / use a GO that is itself the objective (rune, lever,
    // altar, coffin — RequiredNpcOrGo with a negative entry).
    bool TryUseQuestGO(ObjectGuid& pursuedGO, float searchRange = 60.0f);

    // Fire a quest item's OnUse spell at the right target: a spell-focus
    // GO (moonwell), a required creature, or the bot itself.
    bool TryUseQuestItem(ObjectGuid& pursuedGO, ObjectGuid& pursuedTarget, float searchRange = 60.0f);

    // True when a quest-relevant mob is within range — used during
    // travel so we yield to attack-anything instead of running past.
    bool HasNearbyQuestMob(float range = 20.0f);

protected:
    bool GetQuestPOIPosAndObjectiveIdx(uint32 questId, std::vector<POIInfo>& poiInfo, bool toComplete = false);
    static WorldPosition SelectRandomGrindPos(Player* bot);
    static WorldPosition SelectRandomCampPos(Player* bot);
    bool SelectRandomFlightTaxiNode(uint32& flightMasterEntry, WorldPosition& flightMasterPos, std::vector<uint32>& path);
    bool RandomChangeStatus(std::vector<NewRpgStatus> candidateStatus);
    bool CheckRpgStatusAvailable(NewRpgStatus status);

protected:
    /* FOR MOVE FAR */
    // Distance at which MoveFarTo considers the travel-node graph as
    // a routing option. Below this, the move is short enough that
    // mmap handles it directly. Above this, mmap is *still probed
    // first* via the 40-step chained pathfinder; the node graph
    // only takes over if mmap can't get within spellDistance of
    // the destination.
    const float nodeFirstDis = 75.0f;

private:
    void StartTravelPlan(WorldPosition dest);
    bool UpdateTravelPlan();
};

#endif
