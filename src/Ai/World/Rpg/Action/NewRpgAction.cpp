#include "NewRpgAction.h"

#include <cmath>
#include <cstdlib>

#include "AreaDefines.h"
#include "BroadcastHelper.h"
#include "ChatHelper.h"
#include "G3D/Vector2.h"
#include "GossipDef.h"
#include "IVMapMgr.h"
#include "NewRpgInfo.h"
#include "NewRpgStrategy.h"
#include "Object.h"
#include "ObjectAccessor.h"
#include "ObjectDefines.h"
#include "ObjectGuid.h"
#include "ObjectMgr.h"
#include "PathGenerator.h"
#include "Player.h"
#include "PlayerbotAI.h"
#include "Playerbots.h"
#include "QuestDef.h"
#include "Random.h"
#include "SharedDefines.h"
#include "Timer.h"
#include "TravelMgr.h"

bool TellRpgStatusAction::Execute(Event event)
{
    Player* owner = event.getOwner();
    if (!owner)
        return false;
    std::string out = botAI->rpgInfo.ToString();
    bot->Whisper(out.c_str(), LANG_UNIVERSAL, owner);
    return true;
}

bool StartRpgDoQuestAction::Execute(Event event)
{
    Player* owner = event.getOwner();
    if (!owner)
        return false;

    std::string const text = event.getParam();
    PlayerbotChatHandler ch(owner);
    uint32 questId = ch.extractQuestId(text);
    const Quest* quest = sObjectMgr->GetQuestTemplate(questId);
    if (quest)
    {
        botAI->rpgInfo.ChangeToDoQuest(questId, quest);
        bot->Whisper("Start to do quest " + std::to_string(questId), LANG_UNIVERSAL, owner);
        return true;
    }
    bot->Whisper("Invalid quest " + text, LANG_UNIVERSAL, owner);
    return false;
}

bool NewRpgStatusUpdateAction::Execute(Event /*event*/)
{
    NewRpgInfo& info = botAI->rpgInfo;
    NewRpgStatus status = info.GetStatus();
    switch (status)
    {
        case RPG_IDLE:
            return RandomChangeStatus({RPG_GO_CAMP, RPG_GO_GRIND, RPG_WANDER_RANDOM, RPG_WANDER_NPC, RPG_DO_QUEST,
                                       RPG_TRAVEL_FLIGHT, RPG_REST, RPG_OUTDOOR_PVP});

        case RPG_GO_GRIND:
        {
            auto& data = std::get<NewRpgInfo::GoGrind>(info.data);
            WorldPosition& originalPos = data.pos;
            assert(data.pos != WorldPosition());
            // GO_GRIND -> WANDER_RANDOM
            if (bot->GetExactDist(originalPos) < 10.0f)
            {
                info.ChangeToWanderRandom();
                return true;
            }
            break;
        }
        case RPG_GO_CAMP:
        {
            auto& data = std::get<NewRpgInfo::GoCamp>(info.data);
            WorldPosition& originalPos = data.pos;
            assert(data.pos != WorldPosition());
            // GO_CAMP -> WANDER_NPC
            if (bot->GetExactDist(originalPos) < 10.0f)
            {
                info.ChangeToWanderNpc();
                return true;
            }
            break;
        }
        case RPG_WANDER_RANDOM:
        {
            // WANDER_RANDOM -> IDLE
            if (info.HasStatusPersisted(statusWanderRandomDuration))
            {
                info.ChangeToIdle();
                return true;
            }
            break;
        }
        case RPG_WANDER_NPC:
        {
            if (info.HasStatusPersisted(statusWanderNpcDuration))
            {
                info.ChangeToIdle();
                return true;
            }
            break;
        }
        case RPG_DO_QUEST:
        {
            // DO_QUEST -> IDLE
            if (info.HasStatusPersisted(statusDoQuestDuration))
            {
                info.ChangeToIdle();
                return true;
            }
            break;
        }
        // RPG_TRAVEL_FLIGHT arrival is handled inside NewRpgTravelFlightAction
        // so the flight action owns both take-off and landing transitions.
        case RPG_REST:
        {
            // REST -> IDLE
            if (info.HasStatusPersisted(statusRestDuration))
            {
                info.ChangeToIdle();
                return true;
            }
            break;
        }
        case RPG_OUTDOOR_PVP:
        {
            if (info.HasStatusPersisted(statusOutDoorPvPDuration))
            {
                info.ChangeToIdle();
                return true;
            }
            break;
        }
        default:
            break;
    }
    return false;
}

bool NewRpgGoGrindAction::Execute(Event /*event*/)
{
    if (SearchQuestGiverAndAcceptOrReward())
        return true;
    if (auto* data = std::get_if<NewRpgInfo::GoGrind>(&botAI->rpgInfo.data))
    {
        if (MoveFarTo(data->pos))
        {
            botAI->rpgInfo.moveRetryCount = 0;
            return true;
        }
        // Reference pattern (TravelTarget retry counter): count
        // consecutive MoveFarTo failures, give up after N tries by
        // transitioning out of the stuck state instead of nudging in
        // place. Idle lets the status picker rotate to a new state.
        if (++botAI->rpgInfo.moveRetryCount >= NewRpgInfo::MAX_MOVE_RETRIES)
            botAI->rpgInfo.ChangeToIdle();
        return true;  // consume tick, no nudge
    }

    return false;
}

bool NewRpgGoCampAction::Execute(Event /*event*/)
{
    if (SearchQuestGiverAndAcceptOrReward())
        return true;

    if (auto* data = std::get_if<NewRpgInfo::GoCamp>(&botAI->rpgInfo.data))
    {
        if (MoveFarTo(data->pos))
        {
            botAI->rpgInfo.moveRetryCount = 0;
            return true;
        }
        if (++botAI->rpgInfo.moveRetryCount >= NewRpgInfo::MAX_MOVE_RETRIES)
            botAI->rpgInfo.ChangeToIdle();
        return true;
    }

    return false;
}

bool NewRpgWanderRandomAction::Execute(Event /*event*/)
{
    if (SearchQuestGiverAndAcceptOrReward())
        return true;

    return MoveRandomNear();
}

bool NewRpgWanderNpcAction::Execute(Event /*event*/)
{
    NewRpgInfo& info = botAI->rpgInfo;
    auto* dataPtr = std::get_if<NewRpgInfo::WanderNpc>(&info.data);
    if (!dataPtr)
        return false;
    auto& data = *dataPtr;
    if (!data.npcOrGo)
    {
        // No npc can be found, switch to IDLE
        ObjectGuid npcOrGo = ChooseNpcOrGameObjectToInteract();
        if (npcOrGo.IsEmpty())
        {
            info.ChangeToIdle();
            return true;
        }
        data.npcOrGo = npcOrGo;
        data.lastReach = 0;
        return true;
    }

    WorldObject* object = ObjectAccessor::GetWorldObject(*bot, data.npcOrGo);
    if (object && IsWithinInteractionDist(object))
    {
        if (!data.lastReach)
        {
            data.lastReach = getMSTime();
            if (bot->CanInteractWithQuestGiver(object))
                InteractWithNpcOrGameObjectForQuest(data.npcOrGo);
            return true;
        }

        if (data.lastReach && GetMSTimeDiffToNow(data.lastReach) < npcStayTime)
            return false;

        // has reached the npc for more than `npcStayTime`, select the next target
        data.npcOrGo = ObjectGuid();
        data.lastReach = 0;
    }
    else
    {
        if (MoveWorldObjectTo(data.npcOrGo))
        {
            botAI->rpgInfo.moveRetryCount = 0;
            return true;
        }
        // Retry counter (reference pattern): give up after N failures
        // by clearing the picked NPC so next tick picks a different
        // one. No nudge — stand still until retry.
        if (++botAI->rpgInfo.moveRetryCount >= NewRpgInfo::MAX_MOVE_RETRIES)
        {
            data.npcOrGo = ObjectGuid();
            data.lastReach = 0;
            botAI->rpgInfo.moveRetryCount = 0;
        }
        return true;
    }

    return true;
}

bool NewRpgDoQuestAction::Execute(Event /*event*/)
{
    if (SearchQuestGiverAndAcceptOrReward())
        return true;

    NewRpgInfo& info = botAI->rpgInfo;
    auto* dataPtr = std::get_if<NewRpgInfo::DoQuest>(&info.data);
    if (!dataPtr)
        return false;
    auto& data = *dataPtr;
    uint32 questId = data.questId;
    uint8 questStatus = bot->GetQuestStatus(questId);
    switch (questStatus)
    {
        case QUEST_STATUS_INCOMPLETE:
            return DoIncompleteQuest(data);
        case QUEST_STATUS_COMPLETE:
            return DoCompletedQuest(data);
        default:
            break;
    }
    info.ChangeToIdle();
    return true;
}

bool NewRpgDoQuestAction::DoIncompleteQuest(NewRpgInfo::DoQuest& data)
{
    uint32 const questId = data.questId;

    // === Spawn-index pipeline ===
    // Reference (cmangos) per-spawn pattern: walk to specific known
    // spawns of the current objective one by one, advance through the
    // candidate list on per-spawn timeout, refresh the list when the
    // objective makes progress (so the list reflects what's still
    // needed). No POI cluster roam, no random nudging.

    // 1. Detect objective completion. If the current objective is done,
    //    drop the cached spawn list so we re-fetch for the next
    //    incomplete objective on this tick.
    if (!data.candidateSpawns.empty())
    {
        int32 const currentObjective = data.objectiveIdx;
        Quest const* quest = sObjectMgr->GetQuestTemplate(questId);
        QuestStatusData const& q_status = bot->getQuestStatusMap().at(questId);
        bool completed = true;
        if (currentObjective < QUEST_OBJECTIVES_COUNT)
        {
            if (q_status.CreatureOrGOCount[currentObjective] < quest->RequiredNpcOrGoCount[currentObjective])
                completed = false;
        }
        else if (currentObjective < QUEST_OBJECTIVES_COUNT + QUEST_ITEM_OBJECTIVES_COUNT)
        {
            if (q_status.ItemCount[currentObjective - QUEST_OBJECTIVES_COUNT] <
                quest->RequiredItemCount[currentObjective - QUEST_OBJECTIVES_COUNT])
                completed = false;
        }
        if (completed)
        {
            data.candidateSpawns.clear();
            data.currentSpawnIdx = 0;
            data.lastReachPOI = 0;
            data.objectiveIdx = 0;
            data.pursuedLootGO.Clear();
            data.pursuedUseGO.Clear();
            data.pursuedUseTarget.Clear();
        }
    }

    // 2. Fetch spawn candidates if we don't have any. Abandon the
    //    quest if no spawns are indexed on the bot's current map (the
    //    quest is for another zone or our index is missing them).
    if (data.candidateSpawns.empty())
    {
        std::vector<WorldPosition> spawns;
        int32 objectiveIdx = 0;
        if (!FetchQuestSpawnsForObjective(questId, spawns, objectiveIdx))
        {
            botAI->lowPriorityQuest.insert(questId);
            botAI->rpgStatistic.questAbandoned++;
            LOG_DEBUG("playerbots", "[New RPG] {} abandoned quest {} — no spawns indexed",
                      bot->GetName(), questId);
            botAI->rpgInfo.ChangeToIdle();
            return true;
        }
        data.candidateSpawns = std::move(spawns);
        data.currentSpawnIdx = 0;
        data.lastReachPOI = 0;
        data.objectiveIdx = objectiveIdx;
        data.pursuedLootGO.Clear();
        data.pursuedUseGO.Clear();
        data.pursuedUseTarget.Clear();
    }

    // 3. If we've exhausted the candidate list, abandon (the spawn
    //    list was sorted by distance and we tried each).
    if (data.currentSpawnIdx >= data.candidateSpawns.size())
    {
        botAI->lowPriorityQuest.insert(questId);
        botAI->rpgStatistic.questAbandoned++;
        LOG_DEBUG("playerbots", "[New RPG] {} abandoned quest {} — exhausted all {} candidate spawns",
                  bot->GetName(), questId, static_cast<uint32>(data.candidateSpawns.size()));
        botAI->rpgInfo.ChangeToIdle();
        return true;
    }

    WorldPosition const& target = data.candidateSpawns[data.currentSpawnIdx];

    // 4. Walk to the current target spawn. Yield to attack-anything
    //    only if a quest mob for this specific objective is adjacent
    //    (so we don't walk past the target we just spawned next to).
    if (bot->GetDistance(target) > 10.0f && !data.lastReachPOI)
    {
        if (HasNearbyQuestMobForObjective(15.0f, data.questId, data.objectiveIdx))
            return false;

        if (MoveFarTo(target))
        {
            botAI->rpgInfo.moveRetryCount = 0;
            return true;
        }
        // Retry counter: on N consecutive MoveFarTo failures, advance
        // to the next candidate spawn rather than sit on an unreachable
        // one. If that exhausts the list the abandon branch above
        // catches it next tick.
        if (++botAI->rpgInfo.moveRetryCount >= NewRpgInfo::MAX_MOVE_RETRIES)
        {
            ++data.currentSpawnIdx;
            data.lastReachPOI = 0;
            botAI->rpgInfo.moveRetryCount = 0;
        }
        return true;
    }

    // 5. At the spawn. Stamp arrival on first reach so the per-spawn
    //    timeout below has a baseline.
    if (!data.lastReachPOI)
    {
        data.lastReachPOI = getMSTime();
        return true;
    }

    // 6. Per-spawn timeout. The reference's TravelTarget expires after
    //    a configurable window; we use 30s — long enough to finish a
    //    melee pull, short enough to advance off an empty/dead spawn.
    //    On any progression since the list was fetched, refresh so we
    //    re-sort by distance and pick the next nearest live spawn.
    constexpr uint32 perSpawnTimeoutMs = 30 * 1000;
    if (GetMSTimeDiffToNow(data.lastReachPOI) >= perSpawnTimeoutMs)
    {
        bool hasProgression = false;
        int32 const currentObjective = data.objectiveIdx;
        Quest const* quest = sObjectMgr->GetQuestTemplate(questId);
        QuestStatusData const& q_status = bot->getQuestStatusMap().at(questId);
        if (currentObjective < QUEST_OBJECTIVES_COUNT)
        {
            if (q_status.CreatureOrGOCount[currentObjective] != 0 && quest->RequiredNpcOrGoCount[currentObjective])
                hasProgression = true;
        }
        else if (currentObjective < QUEST_OBJECTIVES_COUNT + QUEST_ITEM_OBJECTIVES_COUNT)
        {
            if (q_status.ItemCount[currentObjective - QUEST_OBJECTIVES_COUNT] != 0 &&
                quest->RequiredItemCount[currentObjective - QUEST_OBJECTIVES_COUNT])
                hasProgression = true;
        }
        if (hasProgression)
        {
            // Refresh: re-fetch candidates so the list reflects what's
            // still needed and is sorted from the bot's new position.
            data.candidateSpawns.clear();
            data.currentSpawnIdx = 0;
            data.lastReachPOI = 0;
            return true;
        }
        // No progression at this spawn — advance to the next candidate.
        ++data.currentSpawnIdx;
        data.lastReachPOI = 0;
        data.pursuedLootGO.Clear();
        data.pursuedUseGO.Clear();
        data.pursuedUseTarget.Clear();
        return true;
    }

    // 7. At spawn, within timeout: drive toward specific objectives.
    //    Combat strategy engages adjacent quest mobs; loot/use
    //    actions handle quest GOs and quest items.
    if (TryUseQuestItem(data.pursuedUseGO, data.pursuedUseTarget))
        return true;
    if (TryLootQuestGO(data.pursuedLootGO))
        return true;
    if (TryUseQuestGO(data.pursuedUseGO))
        return true;

    // Yield this tick to combat/grind. No POI roam, no MoveRandomNear:
    // bot stays at the spawn until either combat engages or the
    // per-spawn timeout expires.
    return false;
}

bool NewRpgDoQuestAction::DoCompletedQuest(NewRpgInfo::DoQuest& data)
{
    uint32 questId = data.questId;
    const Quest* quest = data.quest;

    if (data.objectiveIdx != -1)
    {
        // if quest is completed, back to poi with -1 idx to reward
        BroadcastHelper::BroadcastQuestUpdateComplete(botAI, bot, quest);
        botAI->rpgStatistic.questCompleted++;
        std::vector<POIInfo> poiInfo;
        if (!GetQuestPOIPosAndObjectiveIdx(questId, poiInfo, true))
        {
            // can't find a poi pos to reward, stop doing quest for now
            botAI->rpgInfo.ChangeToIdle();
            return false;
        }
        assert(poiInfo.size() > 0);
        // now we get the place to get rewarded
        float dx = poiInfo[0].pos.x, dy = poiInfo[0].pos.y;
        // z = MAX_HEIGHT as we do not know accurate z
        float dz = std::max(bot->GetMap()->GetHeight(dx, dy, MAX_HEIGHT), bot->GetMap()->GetWaterLevel(dx, dy));

        // double check for GetQuestPOIPosAndObjectiveIdx
        if (dz == INVALID_HEIGHT || dz == VMAP_INVALID_HEIGHT_VALUE)
            return false;

        WorldPosition pos(bot->GetMapId(), dx, dy, dz);
        data.lastReachPOI = 0;
        data.pos = pos;
        data.objectiveIdx = -1;

        // Drop the spline + lastPath that DoIncompleteQuest committed
        // to the now-completed objective. Without this, MoveFarTo on
        // the next tick hits the bot->isMoving() / lastPath-reuse
        // early-exits at the top of MoveFarTo and rides the stale
        // path instead of replanning toward the turn-in POI. (This
        // is what `.playerbot bot self` masks by recreating the AI.)
        bot->GetMotionMaster()->Clear();
        AI_VALUE(LastMovement&, "last movement").clear();
    }

    if (data.pos == WorldPosition())
        return false;

    if (bot->GetDistance(data.pos) > 10.0f && !data.lastReachPOI)
    {
        if (MoveFarTo(data.pos))
        {
            botAI->rpgInfo.moveRetryCount = 0;
            return true;
        }
        // Retry counter (reference pattern): mark quest as abandoned
        // if turn-in POI is unreachable repeatedly so the bot doesn't
        // sit on a broken handler.
        if (++botAI->rpgInfo.moveRetryCount >= NewRpgInfo::MAX_MOVE_RETRIES)
            botAI->rpgInfo.ChangeToIdle();
        return true;
    }

    // Now we are near the qoi of reward
    // the quest should be rewarded by SearchQuestGiverAndAcceptOrReward
    if (!data.lastReachPOI)
    {
        data.lastReachPOI = getMSTime();
        return true;
    }
    // stayed at this POI for more than 5 minutes
    if (GetMSTimeDiffToNow(data.lastReachPOI) >= poiStayTime)
    {
        // e.g. Can not reward quest to gameobjects
        /// @TODO: It may be better to make lowPriorityQuest a global set shared by all bots (or saved in db)
        botAI->lowPriorityQuest.insert(questId);
        botAI->rpgStatistic.questAbandoned++;
        LOG_DEBUG("playerbots", "[New RPG] {} marked as abandoned quest {}", bot->GetName(), questId);
        botAI->rpgInfo.ChangeToIdle();
        return true;
    }
    // waiting for SearchQuestGiverAndAcceptOrReward to pick up the NPC;
    // wander instead of false so we don't fall through to grind
    return MoveRandomNear(15.0f);
}

bool NewRpgTravelFlightAction::Execute(Event /*event*/)
{
    NewRpgInfo& info = botAI->rpgInfo;
    auto* dataPtr = std::get_if<NewRpgInfo::TravelFlight>(&info.data);
    if (!dataPtr)
        return false;

    auto& data = *dataPtr;

    // Arrival: we had boarded a flight (data.inFlight) and we're no longer in
    // it → we just landed. Special-case Rut'theran: walk to the portal GO so
    // it teleports the bot into Darnassus, flipping the zone to AREA_DARNASSUS
    // so this branch falls through to ChangeToIdle on the next tick.
    if (data.inFlight && !bot->IsInFlight())
    {
        if (bot->GetZoneId() == AREA_TELDRASSIL)
        {
            static WorldPosition const rutTheranPortalEntrance(1, 8799.41f, 969.787f, 26.2409f, 0.0f);
            return MoveFarTo(rutTheranPortalEntrance);
        }
        info.ChangeToIdle();
        return true;
    }

    if (bot->IsInFlight())
    {
        data.inFlight = true;
        return false;
    }

    if (bot->GetDistance(data.flightMasterPos) > INTERACTION_DISTANCE)
        return MoveFarTo(data.flightMasterPos);

    Creature* flightMaster = bot->FindNearestCreature(data.flightMasterEntry, INTERACTION_DISTANCE * 3);
    if (!flightMaster || !flightMaster->IsAlive())
    {
        info.ChangeToIdle();
        return true;
    }

    if (!TakeFlight(data.path, flightMaster))
    {
        info.ChangeToIdle();
        return true;
    }
    return true;
}
