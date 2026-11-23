import 'package:flutter/cupertino.dart' show CupertinoSliverRefreshControl;
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';

import '../models/history_record.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';

/// REQ-APP-04 (P1): 청소 완료 기록을 리스트로 조회.
class HistoryScreen extends StatefulWidget {
  const HistoryScreen({super.key});

  @override
  State<HistoryScreen> createState() => _HistoryScreenState();
}

class _HistoryScreenState extends State<HistoryScreen> {
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    await context.read<RobotController>().refreshHistory();
    if (mounted) setState(() => _loading = false);
  }

  @override
  Widget build(BuildContext context) {
    final dateFormat = DateFormat('yyyy-MM-dd HH:mm');

    return Scaffold(
      // 리스트가 유리 상단 바 아래로 스크롤되며 비쳐 보이도록 한다.
      extendBodyBehindAppBar: true,
      appBar: const AppTopBar(title: '청소 기록'),
      body: Consumer<RobotController>(
        builder: (context, controller, _) {
          final topPadding = MediaQuery.paddingOf(context).top + 12;
          final bottomPadding = MediaQuery.paddingOf(context).bottom + 24;

          final Widget body;
          if (_loading) {
            body = const SliverFillRemaining(
              hasScrollBody: false,
              child: Center(child: CupertinoActivityIndicator(radius: 14)),
            );
          } else if (controller.history.isEmpty) {
            body = const SliverFillRemaining(
              hasScrollBody: false,
              child: Center(
                child: Text('아직 청소 기록이 없습니다.', style: AppText.callout),
              ),
            );
          } else {
            final records = controller.history;
            body = SliverPadding(
              padding: EdgeInsets.fromLTRB(16, 0, 16, bottomPadding),
              sliver: SliverList.separated(
                itemCount: records.length,
                separatorBuilder: (_, __) => const SizedBox(height: 12),
                itemBuilder: (context, index) => SpringEntrance(
                  // 위쪽 몇 개만 순차적으로 떠오르게 해 과하지 않게 한다.
                  delay: index < 8
                      ? Duration(milliseconds: 45 * index)
                      : Duration.zero,
                  child: _HistoryTile(
                    record: records[index],
                    dateFormat: dateFormat,
                  ),
                ),
              ),
            );
          }

          return CustomScrollView(
            physics: const BouncingScrollPhysics(
              parent: AlwaysScrollableScrollPhysics(),
            ),
            slivers: [
              SliverToBoxAdapter(child: SizedBox(height: topPadding)),
              CupertinoSliverRefreshControl(onRefresh: _load),
              body,
            ],
          );
        },
      ),
    );
  }
}

class _HistoryTile extends StatelessWidget {
  const _HistoryTile({required this.record, required this.dateFormat});

  final HistoryRecord record;
  final DateFormat dateFormat;

  @override
  Widget build(BuildContext context) {
    const placeholder = Icon(
      CupertinoIcons.map,
      size: 22,
      color: AppColors.inkSecondary,
    );

    return AppCard(
      padding: const EdgeInsets.all(14),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ClipPath(
            clipper: ShapeBorderClipper(shape: squircle(16)),
            clipBehavior: Clip.antiAlias,
            child: Container(
              width: 52,
              height: 52,
              color: AppColors.fillSubtle,
              alignment: Alignment.center,
              child: record.mapImageUrl != null
                  ? Image.network(
                      record.mapImageUrl!,
                      fit: BoxFit.cover,
                      width: 52,
                      height: 52,
                      errorBuilder: (_, __, ___) => placeholder,
                    )
                  : placeholder,
            ),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(dateFormat.format(record.finishedAt),
                    style: AppText.headline),
                const SizedBox(height: 4),
                Text(
                  '소요 시간 ${record.durationLabel} · 장애물 ${record.obstaclesFound}건'
                  '${record.note != null ? '\n${record.note}' : ''}',
                  style: AppText.callout,
                ),
              ],
            ),
          ),
          const SizedBox(width: 8),
          AppChip(
            icon: CupertinoIcons.exclamationmark_triangle,
            label: '${record.obstaclesFound}',
            inverted: record.obstaclesFound > 0,
          ),
        ],
      ),
    );
  }
}
